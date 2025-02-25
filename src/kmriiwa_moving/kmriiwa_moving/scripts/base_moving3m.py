#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def move_base(linear_x, duration):
    pub = rospy.Publisher('/kmriiwa/base/command/cmd_vel', Twist, queue_size=10)
    rospy.sleep(1)  # Ensure the publisher is properly registered
    rate = rospy.Rate(10)  # 10 Hz control loop

    twist = Twist()
    twist.linear.x = linear_x

    start_time = rospy.Time.now()
    while (rospy.Time.now() - start_time).to_sec() < duration:
        pub.publish(twist)
        rate.sleep()

    # Stop the robot
    twist.linear.x = 0
    pub.publish(twist)

if __name__ == '__main__':
    try:
        rospy.init_node('base_control_script', anonymous=True)
        rospy.sleep(1)  # Ensure ROS setup is complete

        # Define movement parameters
        linear_speed = 1  # meters per second 0.2 /0.4/ 0.6/ 0.8/1.0
        distance = 2  # meters
        duration = distance / linear_speed  # Time required to move

        for i in range(160):
            rospy.loginfo(f"Iteration {i+1}: Moving forward")
            move_base(linear_speed, duration)

            rospy.sleep(1)  # Pause before moving back

            rospy.loginfo(f"Iteration {i+1}: Moving backward")
            move_base(-linear_speed, duration)

            rospy.sleep(1)  # Pause before next iteration

    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted, shutting down.")
