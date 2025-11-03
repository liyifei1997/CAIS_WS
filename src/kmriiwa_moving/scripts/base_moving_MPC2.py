#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class SimpleMPCWithOdom:
    def __init__(self):
        # ---- Parameters (set via ROS params or defaults) ----
        self.cmd_vel_topic  = rospy.get_param("~cmd_vel_topic",  "/kmriiwa/base/command/cmd_vel")
        self.odom_topic     = rospy.get_param("~odom_topic",     "/odom")  # e.g., "/odom_fused"
        self.target_dist    = rospy.get_param("~target_distance", 2.0)     # meters
        self.dt             = rospy.get_param("~dt",              0.1)     # seconds (10 Hz)
        self.horizon        = rospy.get_param("~horizon",         10)      # number of steps
        self.v_candidates   = rospy.get_param("~v_candidates",    [0.2, 0.4, 0.6, 0.8, 1.0])
        self.v_min          = rospy.get_param("~v_min",           0.0)
        self.v_max          = rospy.get_param("~v_max",           1.0)
        self.a_max          = rospy.get_param("~a_max",           0.5)     # m/s^2 (slew-rate)

        # cost weights
        self.q_track        = rospy.get_param("~q_track",         1.0)     # distance tracking
        self.r_smooth       = rospy.get_param("~r_smooth",        0.1)     # control effort

        # ---- ROS I/O ----
        self.pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)
        self.rate = rospy.Rate(1.0 / self.dt)

        # ---- State from odom ----
        self.have_odom = False
        self.x = 0.0
        self.y = 0.0
        self.x0 = None
        self.y0 = None

        # keep last command to enforce accel limit
        self.u_last = 0.0

    # ---------------- Odometry ----------------
    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        if not self.have_odom:
            self.x0 = self.x
            self.y0 = self.y
            self.have_odom = True

    def dist_traveled(self):
        if not self.have_odom:
            return 0.0
        return math.hypot(self.x - self.x0, self.y - self.y0)

    # --------------- Dynamics (distance) ---------------
    def propagate_distance(self, d0, u):
        # simple integrator model on distance along path (forward-only)
        u_clamped = max(self.v_min, min(self.v_max, u))
        return d0 + u_clamped * self.dt

    # --------------- Cost over the horizon ---------------
    def cost_of_seq(self, d0, u_seq):
        d = d0
        cost = 0.0
        u_prev = self.u_last
        for u in u_seq:
            # apply accel/slew constraint in prediction
            du_max = self.a_max * self.dt
            u = max(min(u, u_prev + du_max), u_prev - du_max)
            u = max(self.v_min, min(self.v_max, u))

            d = self.propagate_distance(d, u)
            # tracking: want d -> target_dist
            cost += self.q_track * (self.target_dist - d) ** 2
            # smoothness: penalize large velocities
            cost += self.r_smooth * (u ** 2)
            u_prev = u
        return cost

    # --------------- Tiny "optimizer" (grid search) ---------------
    def pick_control(self, d_now):
        # try sequences made from candidate speeds; take only first control after eval
        best_seq = None
        best_cost = float("inf")

        # To keep it light, we don’t search all combinations (c^N is huge).
        # We build sequences by repeating one candidate (constant speed over horizon),
        # plus a couple of "ramp" patterns around the last command.
        seqs = []

        # constant-speed sequences
        for v in self.v_candidates:
            seqs.append([v] * self.horizon)

        # gentle ramps around last cmd
        near = sorted(self.v_candidates + [self.u_last], key=lambda vv: abs(vv - self.u_last))[:3]
        for v in near:
            up = min(self.v_max, v + self.a_max * self.dt)
            dn = max(self.v_min, v - self.a_max * self.dt)
            seqs.append([v] * (self.horizon // 2) + [up] * (self.horizon - self.horizon // 2))
            seqs.append([v] * (self.horizon // 2) + [dn] * (self.horizon - self.horizon // 2))

        for seq in seqs:
            c = self.cost_of_seq(d_now, seq)
            if c < best_cost:
                best_cost = c
                best_seq = seq

        # first input of the best sequence
        u0 = best_seq[0] if best_seq else 0.0

        # enforce accel limit for the real command
        du_max = self.a_max * self.dt
        u_cmd = max(min(u0, self.u_last + du_max), self.u_last - du_max)
        u_cmd = max(self.v_min, min(self.v_max, u_cmd))
        return u_cmd

    # --------------- ROS command ----------------
    def send_vel(self, v):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = 0.0
        self.pub.publish(msg)

    # --------------- Run loop ----------------
    def run(self):
        rospy.loginfo("Waiting for odometry...")
        t0 = rospy.Time.now()
        while not rospy.is_shutdown() and not self.have_odom:
            if (rospy.Time.now() - t0).to_sec() > 5.0:
                rospy.logwarn("No odometry received yet. Still waiting...")
                t0 = rospy.Time.now()
            self.rate.sleep()

        rospy.loginfo("Odometry locked. Starting MPC to 2 m.")
        while not rospy.is_shutdown():
            d = self.dist_traveled()
            err = self.target_dist - d
            rospy.loginfo_throttle(1.0, f"Distance: {d:.3f} m, remaining: {err:.3f} m, v_last: {self.u_last:.2f} m/s")

            if d >= self.target_dist:
                break

            # pick control from MPC
            u = self.pick_control(d)

            # publish and update last
            self.send_vel(u)
            self.u_last = u

            self.rate.sleep()

        # stop
        self.send_vel(0.0)
        rospy.loginfo("Target reached. Stopped.")

def main():
    rospy.init_node("simple_mpc_with_odom", anonymous=True)
    ctrl = SimpleMPCWithOdom()
    try:
        ctrl.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
