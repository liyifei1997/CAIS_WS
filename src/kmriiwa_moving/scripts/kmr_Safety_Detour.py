#!/usr/bin/env python3
import math, time, rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState

def yaw_from_quat(q):
    siny_cosp = 2.0*(q.w*q.z + q.x*q.y)
    cosy_cosp = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class Avoid90TwoLasers:
    def __init__(self):
        # ===== topics =====
        self.front_scan_topic = rospy.get_param('~front_scan_topic',
                                                '/kmriiwa/base/state/LaserB1Scan')
        self.side_scan_topic  = rospy.get_param('~side_scan_topic',
                                                '/kmriiwa/base/state/LaserB4Scan')
        self.cmd_vel_topic    = rospy.get_param('~cmd_vel_topic',
                                                '/kmriiwa/base/command/cmd_vel')
        self.model_name       = rospy.get_param('~model_name', 'kmriiwa')

        # ===== speeds =====
        self.v_forward  = rospy.get_param('~v_forward', 0.50)
        self.v_slow     = rospy.get_param('~v_slow',    0.20)
        self.v_strafe   = rospy.get_param('~v_strafe',  0.30)
        self.w_turn     = rospy.get_param('~w_turn',    0.60)

        # ===== front ranges =====
        self.slow_range        = rospy.get_param('~slow_range', 1.00)
        self.stop_range        = rospy.get_param('~stop_range', 0.50)
        self.hysteresis        = rospy.get_param('~hysteresis', 0.10)
        self.front_window_deg  = rospy.get_param('~front_window_deg', 20.0)
        self.front_angle_deg   = rospy.get_param('~front_angle_deg', 0.0)

        # ===== side ranges =====
        # trigger: if side < side_block_range → start STRAFE
        self.side_block_range  = rospy.get_param('~side_block_range', 0.80)
        # we will EXIT strafe early if side > side_clear_range
        self.side_clear_range  = rospy.get_param('~side_clear_range', 0.50)
        self.side_clear_hyst   = rospy.get_param('~side_clear_hyst', 0.05)
        # time cap: don't strafe forever even if it stays blocked
        self.side_strafe_time  = rospy.get_param('~side_strafe_time', 1.5)
        self.side_window_deg   = rospy.get_param('~side_window_deg', 20.0)
        # +1 = B4 on right → move left
        self.side_dir_sign     = rospy.get_param('~side_dir_sign', +1.0)

        # ===== side “emergency slide” thresholds =====
        self.side_emergency_range = rospy.get_param('~side_emergency_range', 0.20)  # start STRAFE
        self.side_safe_range      = rospy.get_param('~side_safe_range',     0.40)  # stop STRAFE
        self.side_clear_hyst      = rospy.get_param('~side_clear_hyst',     0.02)  # small hysteresis
        self.side_strafe_max_time = rospy.get_param('~side_strafe_max_time', 1.5)  # safety cap

        # Place B4’s window so it doesn’t overlap B1’s
        # (example: B1 covers ±30°, B4 (right) covers 30°..90° -> center ~60°)
        self.side_angle_deg = rospy.get_param('~side_angle_deg', 60.0 * self.side_dir_sign)


        # ===== turning =====
        self.turn_angle_deg    = rospy.get_param('~turn_angle_deg', 90.0)
        self.turn_extra_deg    = rospy.get_param('~turn_extra_deg', 5.0)
        self.min_turn_time     = rospy.get_param('~min_turn_time', 0.8)

        # ===== behavior =====
        self.stop_on_nodata    = rospy.get_param('~stop_on_nodata', True)
        self.enable_fallback   = rospy.get_param('~enable_fallback', True)
        self.log_throttle_s    = rospy.get_param('~log_throttle_s', 0.5)

        # ===== state =====
        self.mode = 'CRUISE'  # CRUISE | SLOW | STRAFE | TURN
        self.have_front = False
        self.have_side  = False
        self.front_nearest = float('inf')
        self.side_nearest  = float('inf')
        self.turn_until   = 0.0
        self.strafe_until = 0.0

        # ===== IO =====
        self.pub_cmd = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        rospy.Subscriber(self.front_scan_topic, LaserScan, self.on_front_scan, queue_size=1)
        rospy.Subscriber(self.side_scan_topic,  LaserScan, self.on_side_scan,  queue_size=1)

        # gazebo services
        self.get_state = None
        self.set_state = None

        rospy.loginfo("Avoid90TwoLasers: front=%s side=%s cmd=%s",
                      self.front_scan_topic, self.side_scan_topic, self.cmd_vel_topic)

        # --- add this in __init__ with the other params ---
        self.side_angle_deg = rospy.get_param('~side_angle_deg', 90.0 * self.side_dir_sign)

        


    # ---------- utils ----------
    @staticmethod
    def _finite_in_range(r, rmin, rmax):
        return (r == r) and (rmin < r < rmax)

    def _scan_nearest_in_window(self, msg, center_deg, window_deg):
        if msg.angle_increment == 0.0:
            return float('inf')

        center = math.radians(center_deg)
        half   = math.radians(window_deg)

        if not (msg.angle_min <= center <= msg.angle_max):
            return float('inf')

        mid_idx   = int(round((center - msg.angle_min)/msg.angle_increment))
        half_bins = max(1, int(half / abs(msg.angle_increment)))
        i0 = max(0, mid_idx - half_bins)
        i1 = min(len(msg.ranges), mid_idx + half_bins + 1)

        nearest = float('inf')
        for r in msg.ranges[i0:i1]:
            if self._finite_in_range(r, msg.range_min, msg.range_max):
                if r < nearest:
                    nearest = r
        return nearest

    # ---------- callbacks ----------
    def on_front_scan(self, msg: LaserScan):
        self.have_front = True
        self.front_nearest = self._scan_nearest_in_window(
            msg, self.front_angle_deg, self.front_window_deg
        )

    def on_side_scan(self, msg: LaserScan):
        self.have_side = True
        self.side_nearest = self._scan_nearest_in_window(
            msg, self.side_angle_deg, self.side_window_deg
        )

    # ---------- gazebo fallback ----------
    def _ensure_services(self):
        if self.get_state and self.set_state:
            return True
        try:
            rospy.wait_for_service('/gazebo/get_model_state', timeout=2.0)
            rospy.wait_for_service('/gazebo/set_model_state', timeout=2.0)
            self.get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            rospy.loginfo_once("Fallback enabled: using /gazebo/* services.")
            return True
        except Exception as e:
            rospy.logwarn_throttle(2.0, "Fallback unavailable: %s", e)
            return False

    def _publish_cmd(self, vx_robot, vy_robot, w):
        tw = Twist()
        tw.linear.x  = vx_robot
        tw.linear.y  = vy_robot
        tw.angular.z = w

        if self.pub_cmd.get_num_connections() > 0:
            self.pub_cmd.publish(tw)
            return

        if not self.enable_fallback:
            return
        if not self._ensure_services():
            return

        try:
            st = self.get_state(self.model_name, 'world')
            yaw = yaw_from_quat(st.pose.orientation)

            vx_w = vx_robot * math.cos(yaw) - vy_robot * math.sin(yaw)
            vy_w = vx_robot * math.sin(yaw) + vy_robot * math.cos(yaw)

            ms = ModelState()
            ms.model_name = self.model_name
            ms.pose = st.pose
            ms.reference_frame = "world"
            ms.twist.linear.x  = vx_w
            ms.twist.linear.y  = vy_w
            ms.twist.angular.z = w
            self.set_state(ms)
        except Exception as e:
            rospy.logwarn_throttle(2.0, "Gazebo service drive error: %s", e)

    # ---------- mode logic ----------
    def _update_mode(self):
        now = rospy.Time.now().to_sec()

        # Keep TURN until time is up
        if self.mode == 'TURN':
            if now >= self.turn_until:
                self.mode = 'CRUISE'
            return

        # STRAFE: exit when side is safe enough or timer elapsed
        if self.mode == 'STRAFE':
            if (self.have_side and
                self.side_nearest >= (self.side_safe_range + self.side_clear_hyst)):
                # hand control back to B1 immediately
                if self.have_front and self.front_nearest <= self.slow_range:
                    self.mode = 'SLOW'
                else:
                    self.mode = 'CRUISE'
                return

            if now >= self.strafe_until:
                # timeout safety: hand back to B1
                if self.have_front and self.front_nearest <= self.slow_range:
                    self.mode = 'SLOW'
                else:
                    self.mode = 'CRUISE'
                return

            # else keep strafing
            return

        # From here: not TURN, not STRAFE
        if not self.have_front:
            # If no front data: either keep going or do a cautious TURN (your choice)
            self.mode = 'CRUISE' if not self.stop_on_nodata else 'TURN'
            # If we set TURN on nodata, precompute its end time
            if self.mode == 'TURN':
                angle = math.radians(abs(self.turn_angle_deg) + self.turn_extra_deg)
                tturn = max(self.min_turn_time, angle / max(1e-3, abs(self.w_turn)))
                self.turn_until = now + tturn
            return

        d_front = self.front_nearest
        d_side  = self.side_nearest if self.have_side else float('inf')

        # ---- B1 (front) has priority ----
        # 1) Emergency TURN if very close ahead
        if d_front <= self.stop_range:
            angle = math.radians(abs(self.turn_angle_deg) + self.turn_extra_deg)
            tturn = max(self.min_turn_time, angle / max(1e-3, abs(self.w_turn)))
            self.turn_until = now + tturn
            self.mode = 'TURN'
            return

        # ---- B4 (side) emergency slide (lower priority than front TURN) ----
        # If something is dangerously close on the B4 side, STRAFE away briefly.
        if self.have_side and d_side <= self.side_emergency_range:
            self.strafe_until = now + self.side_strafe_max_time
            self.mode = 'STRAFE'
            return

        # 2) B1 slow band
        if d_front <= self.slow_range:
            self.mode = 'SLOW'
            return

        # 3) Default cruise
        self.mode = 'CRUISE'

    # ---------- main ----------
    def spin(self):
        last_log = 0.0
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self._update_mode()

            if self.mode == 'TURN':
                self._publish_cmd(0.0, 0.0, self.w_turn)
            elif self.mode == 'STRAFE':
                self._publish_cmd(0.0, self.side_dir_sign * self.v_strafe, 0.0)
            elif self.mode == 'SLOW':
                self._publish_cmd(self.v_slow, 0.0, 0.0)
            else:  # CRUISE
                self._publish_cmd(self.v_forward, 0.0, 0.0)

            now = time.time()
            if now - last_log > self.log_throttle_s:
                rospy.loginfo("mode=%s  front=%.3f  side=%.3f",
                              self.mode,
                              (self.front_nearest if self.front_nearest < float('inf') else -1.0),
                              (self.side_nearest  if self.side_nearest  < float('inf') else -1.0))
                last_log = now

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('kmr_avoid90_two_lasers')
    Avoid90TwoLasers().spin()
