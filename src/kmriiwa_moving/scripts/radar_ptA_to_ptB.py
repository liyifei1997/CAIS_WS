#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from kmriiwa_msgs.msg import JointPosition
import math

class TaskSequence:
    def __init__(self):
        rospy.init_node('kmr_path_pick_place')

        self.cmd_pub = rospy.Publisher('/kmriiwa/base/command/cmd_vel', Twist, queue_size=1)
        self.arm_pub = rospy.Publisher('/kmriiwa/arm/command/JointPosition', JointPosition, queue_size=1)
        rospy.Subscriber('/kmriiwa/base/state/LaserB1Scan', LaserScan, self.scan_cb)
        self.tf_listener = tf.TransformListener()

        self.front_dist = float('inf')
        self.stop_thresh = 0.6
        self.forward_speed = 0.2
        self.turn_speed = 0.3
        rospy.sleep(1.0)

    def scan_cb(self, msg):
        if len(msg.ranges) > 270:
            val = msg.ranges[270]
            if not math.isinf(val) and not math.isnan(val):
                self.front_dist = val
            else:
                self.front_dist = float('inf')

    def get_yaw(self):
        try:
            self.tf_listener.waitForTransform("/kmriiwa_odom", "/kmriiwa_base_footprint", rospy.Time(0), rospy.Duration(1.0))
            (_, rot) = self.tf_listener.lookupTransform("/kmriiwa_odom", "/kmriiwa_base_footprint", rospy.Time(0))
            _, _, yaw = tf.transformations.euler_from_quaternion(rot)
            return yaw
        except:
            return None

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def move_until_wall(self):
        twist = Twist()
        twist.linear.x = self.forward_speed
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.front_dist > self.stop_thresh:
            self.cmd_pub.publish(twist)
            rate.sleep()
        self.cmd_pub.publish(Twist())

    def move_forward(self, duration):
        twist = Twist()
        twist.linear.x = self.forward_speed
        self.cmd_pub.publish(twist)
        rospy.sleep(duration)
        self.cmd_pub.publish(Twist())

    def turn_left_90(self):
        start_yaw = self.get_yaw()
        if start_yaw is None:
            return
        twist = Twist()
        twist.angular.z = self.turn_speed
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            current_yaw = self.get_yaw()
            if current_yaw is None:
                continue
            delta = self.normalize_angle(current_yaw - start_yaw)
            if abs(delta) >= math.pi / 2:
                break
            self.cmd_pub.publish(twist)
            rate.sleep()
        self.cmd_pub.publish(Twist())

    def move_arm(self, positions):
        for pos in positions:
            pos.header.stamp = rospy.Time.now()
            self.arm_pub.publish(pos)
            rospy.sleep(3)

    def run_sequence(self):
        # 1. Move until wall
        self.move_until_wall()
        # 2. Pick up object
        self.pick_up()
        # 3. Rotate left 90°
        self.turn_left_90()
        # 4. Move until next wall
        self.move_until_wall()
        # 5. Rotate left
        self.turn_left_90()
        # 6. Move until next wall
        self.move_until_wall()
        # 7. Rotate left
        self.turn_left_90()
        # 8. Move forward (partway)
        self.move_forward(3)  # adjust duration as needed
        # 9. Place object
        self.place_down()

    def pick_up(self):
        original_pos = JointPosition(a1=0.0, a2=0.0, a3=0.0, a4=0.0, a5=0.0, a6=0.0, a7=0.0)
        reach_pos = JointPosition(a1=0, a2=1.9, a3=1, a4=0, a5=0, a6=0, a7=0)
        grab_pos = JointPosition(a1=0, a2=1.9, a3=0, a4=1.5, a5=0, a6=0, a7=0)
        self.move_arm([original_pos, reach_pos, grab_pos])
        rospy.sleep(20) 

    def place_down(self):
        original_pos = JointPosition(a1=0.0, a2=0.0, a3=0.0, a4=0.0, a5=0.0, a6=0.0, a7=0.0)
        grab_pos = JointPosition(a1=0, a2=1.9, a3=0, a4=1.5, a5=0, a6=0, a7=0)
        place_pos = JointPosition(a1=0, a2=1.9, a3=1, a4=0, a5=0, a6=0, a7=0)
        self.move_arm([grab_pos, place_pos, original_pos])
        rospy.sleep(20) 

if __name__ == '__main__':
    try:
        TaskSequence().run_sequence()
    except rospy.ROSInterruptException:
        pass
