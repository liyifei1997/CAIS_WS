#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import time

def move_base(linear_x, linear_y, duration):
    pub = rospy.Publisher('/kmriiwa/base/command/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    twist = Twist()
    twist.linear.x = linear_x
    twist.linear.y = linear_y

    start_time = time.time()
    while time.time() - start_time < duration:
        pub.publish(twist)
        rate.sleep()

    # Stop the robot after moving
    twist.linear.x = 0
    twist.linear.y = 0
    pub.publish(twist)

if __name__ == '__main__':
    try:
        rospy.init_node('base_control_script', anonymous=True)

        # Define the parameters for the movement
        linear_speed = 0.2  # meters per second
        distance = 1.0  # meters
        duration = distance / linear_speed  # seconds

        # Move 1 meter in the x direction
        move_base(linear_speed, 0.0, duration)

        # Sleep for a short time before the next movement
        time.sleep(1)

        # Move 1 meter in the y direction
        move_base(0.0, linear_speed, duration)
        
    except rospy.ROSInterruptException:
        pass
