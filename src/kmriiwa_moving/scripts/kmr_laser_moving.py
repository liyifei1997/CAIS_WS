#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, math, time, actionlib, collections
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan
from kmriiwa_msgs.msg import JointPosition
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import tf2_ros
import tf_conversions

# ==== 配置（如需要可修改）====
TOPIC_ODOM       = "/kmriiwa/base/state/odom"
TOPIC_CMDVEL     = "/kmriiwa/base/command/cmd_vel"
TOPIC_JP_CMD     = "/kmriiwa/arm/command/JointPosition"
TOPIC_JOINTSTS   = "/kmriiwa/arm/joint_states"
ACTION_FJT       = "/kmriiwa/arm/manipulator_controller/follow_joint_trajectory"

TOPIC_LASER_B1   = "/kmriiwa/base/state/LaserB1Scan"
TOPIC_LASER_B4   = "/kmriiwa/base/state/LaserB4Scan"
FRAME_BASE       = "kmriiwa_base_link"          # base frame
FRAME_B1         = "kmriiwa_laser_B1_link"
FRAME_B4         = "kmriiwa_laser_B4_link"

JOINT_NAMES      = ["A1","A2","A3","A4","A5","A6","A7"]
RATE_HZ          = 20
ANGLE_TOL_DEG    = 2.0
PRINT_HZ         = 10.0
SAFE_POSE_DEG    = [-120, 60, 0, -60, 0, 60, 0]

# Obstacle-check params
Y_LATERAL_MAX    = 0.35   # |y|<= this (m) for points "in-lane"
CLEAR_MAX_RANGE  = 6.0    # ignore points farther than this (m)
PERCENTILE_Q     = 20.0   # robust distance (percentile)
TMP_MED_WINDOW   = 5      # temporal median length
STOP_DIST        = 0.20
SLOW_DIST        = 0.50

V_FAST           = 0.40
V_SLOW           = 0.20

MAX_TRAVEL_PER_LEG = 10.0 # safety cap (meters) to avoid infinite travel if no obstacles

def deg2rad(d): return d * math.pi / 180.0
def rad2deg(r): return r * 180.0 / math.pi

def pad_to_len(vals, target_len, pad=0.0):
    vals = list(vals)
    if len(vals) < target_len:
        vals = vals + [pad]*(target_len - len(vals))
    return vals[:target_len]

def is_same_pose_deg(pose_a, pose_b, tol_deg):
    a = pad_to_len(pose_a, len(JOINT_NAMES), 0.0)
    b = pad_to_len(pose_b, len(JOINT_NAMES), 0.0)
    return all(abs(x - y) <= tol_deg for x, y in zip(a, b))

class KMRSeqWithLidar:
    def __init__(self):
        # pubs/subs
        self.pub_cmd   = rospy.Publisher(TOPIC_CMDVEL, Twist, queue_size=10)
        self.pub_jp    = rospy.Publisher(TOPIC_JP_CMD, JointPosition, queue_size=10)
        self.sub_odom  = rospy.Subscriber(TOPIC_ODOM, Odometry, self.odom_cb)
        self.sub_jstat = rospy.Subscriber(TOPIC_JOINTSTS, JointState, self.joint_cb)

        # lidar
        self.sub_b1    = rospy.Subscriber(TOPIC_LASER_B1, LaserScan, self.lidar_b1_cb, queue_size=1)
        self.sub_b4    = rospy.Subscriber(TOPIC_LASER_B4, LaserScan, self.lidar_b4_cb, queue_size=1)

        # action
        self.fjt_client = actionlib.SimpleActionClient(ACTION_FJT, FollowJointTrajectoryAction)

        # odom
        self.have_odom = False
        self.x = 0.0; self.y = 0.0
        self.ref_x = 0.0; self.ref_y = 0.0

        # joints
        self.have_joints = False
        self.joint_map = {}

        # lidar caches
        self.scan_b1 = None
        self.scan_b4 = None

        # tf
        self.tfbuf = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tflis = tf2_ros.TransformListener(self.tfbuf)
        self.have_tf_b1 = False
        self.have_tf_b4 = False

        # temporal median buffers
        self.clear_hist_fwd = collections.deque(maxlen=TMP_MED_WINDOW)
        self.clear_hist_rev = collections.deque(maxlen=TMP_MED_WINDOW)

        self.rate = rospy.Rate(RATE_HZ)
        rospy.on_shutdown(self.stop_base)

    # ---------------- ODOM ----------------
    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.have_odom = True

    def zero_odom(self):
        while not rospy.is_shutdown() and not self.have_odom:
            self.rate.sleep()
        self.ref_x = self.x
        self.ref_y = self.y
        rospy.loginfo("Odom zeroed at (%.3f, %.3f)", self.ref_x, self.ref_y)

    def rel_xy(self):
        return (self.x - self.ref_x, self.y - self.ref_y)

    # ---------------- JOINT STATES ----------------
    def joint_cb(self, msg: JointState):
        for n, p in zip(msg.name, msg.position):
            self.joint_map[n] = p
        if len(self.joint_map) > 0:
            self.have_joints = True

    def current_joints_deg(self):
        vals = []
        for n in JOINT_NAMES:
            if n not in self.joint_map:
                return None
            vals.append(rad2deg(self.joint_map[n]))
        return vals

    def print_joints_10hz_once(self):
        vals = self.current_joints_deg()
        if vals is not None:
            txt = ", ".join(["%s:%6.2f°" % (n, v) for n, v in zip(JOINT_NAMES, vals)])
            rospy.loginfo("[Joints] %s", txt)

    # ---------------- ARM moves ----------------
    def wait_until_reached_deg(self, target_deg, tol_deg=ANGLE_TOL_DEG, timeout_s=10.0, keep_print=True):
        t_end = time.time() + timeout_s
        target = pad_to_len(target_deg, len(JOINT_NAMES), 0.0)
        next_print = 0.0
        r = rospy.Rate(PRINT_HZ)
        diffs = None
        while not rospy.is_shutdown():
            cur = self.current_joints_deg()
            now = time.time()
            if cur is not None:
                diffs = [abs(c - t) for c, t in zip(cur, target)]
                reached = all(d <= tol_deg for d in diffs)
                if keep_print and now >= next_print:
                    self.print_joints_10hz_once()
                    next_print = now + 1.0/PRINT_HZ
                if reached:
                    return True
            if now > t_end:
                rospy.logwarn("Wait timeout (%.1fs). Last diff(deg)=%s",
                              timeout_s,
                              ["%.2f" % d for d in (diffs or [])])
                return False
            r.sleep()

    def move_arm_point_deg(self, degs, hold_cmd_s=1.0, wait_tol_deg=ANGLE_TOL_DEG, wait_timeout_s=12.0):
        vals = pad_to_len(degs, len(JOINT_NAMES), 0.0)
        send_rate = rospy.Rate(20)
        t_end_send = time.time() + hold_cmd_s
        next_print = 0.0
        while not rospy.is_shutdown() and time.time() < t_end_send:
            msg = JointPosition()
            msg.header.stamp = rospy.Time.now()
            rad_vals = [deg2rad(v) for v in vals]
            msg.a1, msg.a2, msg.a3, msg.a4, msg.a5, msg.a6, msg.a7 = rad_vals
            self.pub_jp.publish(msg)
            now = time.time()
            if now >= next_print:
                self.print_joints_10hz_once()
                next_print = now + 1.0/PRINT_HZ
            send_rate.sleep()
        ok = self.wait_until_reached_deg(vals, tol_deg=wait_tol_deg, timeout_s=wait_timeout_s, keep_print=True)
        if ok and is_same_pose_deg(vals, SAFE_POSE_DEG, ANGLE_TOL_DEG):
            rospy.loginfo("Reached SAFE pose. Extra wait 1.0s...")
            rospy.sleep(1.0)
        elif not ok:
            rospy.logwarn("Arm did not reach target within tolerance (%.1f°).", wait_tol_deg)
        return ok

    # ---------------- BASE moves ----------------
    def stop_base(self):
        self.pub_cmd.publish(Twist())

    def cmd_x_speed(self, vx):
        tw = Twist()
        tw.linear.x = vx
        self.pub_cmd.publish(tw)

    # ---------------- LIDAR callbacks ----------------
    def lidar_b1_cb(self, msg: LaserScan):
        self.scan_b1 = msg

    def lidar_b4_cb(self, msg: LaserScan):
        self.scan_b4 = msg

    # ---------------- TF helpers ----------------
    def _lookup_tf(self, child_frame):
        try:
            tr = self.tfbuf.lookup_transform(FRAME_BASE, child_frame, rospy.Time(0), rospy.Duration(0.2))
            trans = tr.transform.translation
            rot = tr.transform.rotation
            q = (rot.x, rot.y, rot.z, rot.w)
            R = tf_conversions.transformations.quaternion_matrix(q)[:3,:3]  # 3x3
            t = np.array([trans.x, trans.y, trans.z])
            return True, R, t
        except Exception as e:
            rospy.logwarn_throttle(1.0, "TF lookup failed %s -> %s: %s", FRAME_BASE, child_frame, str(e))
            return False, None, None

    # ---------------- scan -> clearance ----------------
    def _scan_to_points_base(self, scan: LaserScan, R_bt, t_bt):
        """Return Nx2 array of (x,y) in base_link."""
        if scan is None or R_bt is None or t_bt is None:
            return None
        n = len(scan.ranges)
        ang0 = scan.angle_min
        inc  = scan.angle_increment
        rs   = np.asarray(scan.ranges, dtype=np.float32)

        # drop NaN/inf/out-of-range
        rs[np.isnan(rs)] = np.inf
        rs[~np.isfinite(rs)] = np.inf
        rs[(rs < max(0.0, scan.range_min)) | (rs > min(CLEAR_MAX_RANGE, scan.range_max))] = np.inf

        # bearings in laser frame (2D)
        idx = np.arange(n, dtype=np.float32)
        th  = ang0 + idx*inc
        c, s = np.cos(th), np.sin(th)

        # points in laser frame (ignore z)
        px_l = rs * c
        py_l = rs * s
        p_l  = np.stack([px_l, py_l, np.zeros_like(px_l)], axis=0)  # 3xN

        # transform to base: p_b = R * p_l + t
        p_b = (R_bt @ p_l) + t_bt.reshape(3,1)
        xy  = p_b[:2,:].T  # Nx2
        # remove inf rows (where rs was inf)
        good = np.isfinite(xy).all(axis=1)
        return xy[good]

    def _get_clearance_dir(self, direction_sign):
        """
        direction_sign: +1 (moving +x), -1 (moving -x)
        returns robust clearance (meters) or None if no data
        """
        # ensure TFs
        if not self.have_tf_b1:
            ok1, R1, t1 = self._lookup_tf(FRAME_B1)
            if ok1:
                self.R_b1, self.t_b1 = R1, t1
                self.have_tf_b1 = True
        if not self.have_tf_b4:
            ok4, R4, t4 = self._lookup_tf(FRAME_B4)
            if ok4:
                self.R_b4, self.t_b4 = R4, t4
                self.have_tf_b4 = True

        pts = []
        if self.have_tf_b1 and self.scan_b1 is not None:
            p = self._scan_to_points_base(self.scan_b1, self.R_b1, self.t_b1)
            if p is not None: pts.append(p)
        if self.have_tf_b4 and self.scan_b4 is not None:
            p = self._scan_to_points_base(self.scan_b4, self.R_b4, self.t_b4)
            if p is not None: pts.append(p)
        if len(pts) == 0:
            return None

        P = np.vstack(pts)  # Nx2
        x = P[:,0]; y = P[:,1]

        if direction_sign > 0:
            mask = (x > 0.0) & (np.abs(y) <= Y_LATERAL_MAX)
            d = x[mask]  # forward clearance along +x
        else:
            mask = (x < 0.0) & (np.abs(y) <= Y_LATERAL_MAX)
            d = -x[mask]  # forward clearance along -x (make positive)

        if d.size == 0:
            return None

        # robust percentile (drop tiny residual spikes)
        d_sorted = np.sort(d)
        q = np.percentile(d_sorted, PERCENTILE_Q)

        # small temporal median to smooth flicker
        if direction_sign > 0:
            self.clear_hist_fwd.append(q)
            q_med = float(np.median(self.clear_hist_fwd))
        else:
            self.clear_hist_rev.append(q)
            q_med = float(np.median(self.clear_hist_rev))

        return q_med

    # ---------------- obstacle-aware leg ----------------
    def move_leg_with_avoid(self, direction_sign, leg_name="leg"):
        """
        direction_sign: +1 (+x) or -1 (-x)
        drives until STOP_DIST reached (or max leg distance cap)
        """
        # starting rel x for distance cap
        x0, y0 = self.rel_xy()
        last_log = time.time()

        r = rospy.Rate(RATE_HZ)
        while not rospy.is_shutdown():
            clr = self._get_clearance_dir(direction_sign)
            if clr is None:
                # if no lidar yet, crawl slowly and wait
                v = direction_sign * 0.10
            else:
                if clr <= STOP_DIST:
                    self.stop_base()
                    rospy.loginfo("%s: STOP (clearance=%.2f m)", leg_name, clr)
                    return  # leg done
                elif clr <= SLOW_DIST:
                    v = direction_sign * V_SLOW
                else:
                    v = direction_sign * V_FAST

            self.cmd_x_speed(v)

            # print status at 5 Hz
            if time.time() - last_log > 0.2:
                rx, ry = self.rel_xy()
                rospy.loginfo("%s: v=%.2f m/s, clearance=%s, rel=(%.2f, %.2f)",
                              leg_name, v, "NA" if clr is None else f"{clr:.2f}", rx, ry)
                last_log = time.time()

            # safety travel cap
            rx, _ = self.rel_xy()
            if abs(rx - x0) >= MAX_TRAVEL_PER_LEG:
                self.stop_base()
                rospy.logwarn("%s: reached max travel cap (%.1f m), stopping leg.",
                              leg_name, MAX_TRAVEL_PER_LEG)
                return

            r.sleep()

    # ---------------- main sequence ----------------
    def run(self):
        rospy.loginfo("Waiting for odom + joints + (TF/lasers will come online automatically)...")
        while not rospy.is_shutdown() and (not self.have_odom or not self.have_joints):
            self.rate.sleep()

        # (1) zero odom
        self.zero_odom()
        rospy.loginfo("Task 1: odom zeroed")

        # (2) arm to SAFE pose
        self.move_arm_point_deg(SAFE_POSE_DEG, hold_cmd_s=1.0)
        rospy.loginfo("Task 2: arm safe pose done")

        # (3) 4 cycles: +x then -x, obstacle-aware speeds
        for i in range(4):
            rospy.loginfo("Cycle %d / 4: +x leg", i+1)
            self.move_leg_with_avoid(+1, leg_name=f"cycle{i+1}_+x")
            rospy.sleep(0.3)
            rospy.loginfo("Cycle %d / 4: -x leg", i+1)
            self.move_leg_with_avoid(-1, leg_name=f"cycle{i+1}_-x")
            rospy.sleep(0.3)

        self.stop_base()
        rospy.loginfo("All cycles finished.")

if __name__ == "__main__":
    rospy.init_node("kmr_sequence_with_lidar", anonymous=True)
    KMRSeqWithLidar().run()
