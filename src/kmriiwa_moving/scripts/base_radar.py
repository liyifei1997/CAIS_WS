#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class ReactiveRectangle:
    def __init__(self):
        rospy.init_node('reactive_rectangle_tf', anonymous=True)

        # Publishers and Subscribers
        self.cmd_pub = rospy.Publisher('/kmriiwa/base/command/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/kmriiwa/base/state/LaserB1Scan', LaserScan, self.scan_cb)
        self.tf_listener = tf.TransformListener()

        # Laser & motion settings
        self.front_dist = float('inf')
        self.stop_thresh = 0.6      # stop distance from wall (meters)
        self.forward_speed = 0.2    # m/s
        self.turn_speed = 0.3       # rad/s

        rospy.sleep(1.0)  # wait for tf buffer to fill

    def scan_cb(self, msg):
        if len(msg.ranges) > 270:
            val = msg.ranges[270]
            if not math.isinf(val) and not math.isnan(val):
                self.front_dist = val
            else:
                self.front_dist = float('inf')
        rospy.loginfo("📡 Front laser distance: %.2f m", self.front_dist)

    def get_yaw(self):
        try:
            self.tf_listener.waitForTransform("/kmriiwa_odom", "/kmriiwa_base_footprint", rospy.Time(0), rospy.Duration(1.0))
            (_, rot) = self.tf_listener.lookupTransform("/kmriiwa_odom", "/kmriiwa_base_footprint", rospy.Time(0))
            _, _, yaw = tf.transformations.euler_from_quaternion(rot)
            return yaw
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("⚠️ Could not get yaw from TF")
            return None

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def move_until_obstacle(self):
        twist = Twist()
        twist.linear.x = self.forward_speed
        rate = rospy.Rate(10)

        rospy.loginfo("🚗 Moving forward until obstacle is detected...")
        while not rospy.is_shutdown() and self.front_dist > self.stop_thresh:
            self.cmd_pub.publish(twist)
            rate.sleep()

        self.cmd_pub.publish(Twist())
        rospy.loginfo("🛑 Obstacle detected at %.2f m, stopping.", self.front_dist)

    def turn_90_degrees(self):
        rate = rospy.Rate(20)
        twist = Twist()
        twist.angular.z = self.turn_speed

        start_yaw = self.get_yaw()
        if start_yaw is None:
            return

        rospy.loginfo("🔄 Starting 90° turn from yaw: %.2f", start_yaw)

        while not rospy.is_shutdown():
            current_yaw = self.get_yaw()
            if current_yaw is None:
                continue

            delta_yaw = self.normalize_angle(current_yaw - start_yaw)
            rospy.loginfo("⏱️ Turned: %.2f radians (%.1f°)", delta_yaw, math.degrees(delta_yaw))

            if abs(delta_yaw) >= math.pi / 2:
                break

            self.cmd_pub.publish(twist)
            rate.sleep()

        self.cmd_pub.publish(Twist())
        rospy.loginfo("✅ Turn complete.")

    def run(self):
        rospy.loginfo("🟦 Starting rectangle walk using radar + TF yaw...")
        for i in range(4):
            rospy.loginfo(f"🔁 Side {i+1}/4")
            self.move_until_obstacle()
            rospy.sleep(1)
            self.turn_90_degrees()
            rospy.sleep(1)
        rospy.loginfo("🎉 Rectangle walk complete!")

if __name__ == '__main__':
    try:
        ReactiveRectangle().run()
    except rospy.ROSInterruptException:
        pass
