#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import time

def move_base(linear_x, duration):
    pub = rospy.Publisher('/kmriiwa/base/command/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    twist = Twist()
    twist.linear.x = linear_x

    start_time = time.time()
    while time.time() - start_time < duration:
        pub.publish(twist)
        rate.sleep()

    # Stop the robot after moving
    twist.linear.x = 0
    pub.publish(twist)

if __name__ == '__main__':
    try:
        rospy.init_node('base_control_script', anonymous=True)

        # Define the parameters for the movement
        linear_speed = 0.3  # meters per second
        distance = 1.5  # meters
        duration = distance / linear_speed  # time required to move 3 meters

        for i in range(100):
        # Move 3 meters forward in the x direction
            move_base(linear_speed, duration)

        # Sleep for 1 second before moving back
            time.sleep(1)

        # Move 3 meters backward in the x direction
            move_base(-linear_speed, duration)

            time.sleep(1)

        
        

    except rospy.ROSInterruptException:
        pass
