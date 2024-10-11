#!/usr/bin/env python3

import rospy
from kmriiwa_msgs.msg import JointPosition
from geometry_msgs.msg import Twist
import threading

def publish_arm_commands(arm_pub, joint_pos):
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        joint_pos.header.stamp = rospy.Time.now()
        arm_pub.publish(joint_pos)
        rate.sleep()

def publish_base_commands(base_pub, twist):
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        base_pub.publish(twist)
        rate.sleep()

def main():
    rospy.init_node('robot_control', anonymous=True)

    arm_pub = rospy.Publisher('/kmriiwa/arm/command/JointPosition', JointPosition, queue_size=10)
    base_pub = rospy.Publisher('/kmriiwa/base/command/cmd_vel', Twist, queue_size=10)

    # Create JointPosition message
    joint_pos = JointPosition()
    joint_pos.header.seq = 0
    joint_pos.header.stamp = rospy.Time.now()
    joint_pos.header.frame_id = ''
    joint_pos.a1 = 1.0
    joint_pos.a2 = 0.5
    joint_pos.a3 = -0.5
    joint_pos.a4 = 0.2
    joint_pos.a5 = 0.0
    joint_pos.a6 = 1.5
    joint_pos.a7 = -1.0

    # Create Twist message
    twist = Twist()
    twist.linear.x = 0.0
    twist.linear.y = 0.5
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0

    # Start threads for publishing
    arm_thread = threading.Thread(target=publish_arm_commands, args=(arm_pub, joint_pos))
    base_thread = threading.Thread(target=publish_base_commands, args=(base_pub, twist))

    arm_thread.start()
    base_thread.start()

    arm_thread.join()
    base_thread.join()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
