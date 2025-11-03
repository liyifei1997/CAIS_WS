#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import time

def move_base(pub, linear_x, linear_y, duration):
    rate = rospy.Rate(10)  # 10 Hz
    twist = Twist()
    twist.linear.x = linear_x
    twist.linear.y = linear_y

    start_time = time.time()
    while time.time() - start_time < duration and not rospy.is_shutdown():
        pub.publish(twist)
        rate.sleep()

    # stop after moving
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    pub.publish(twist)

if __name__ == '__main__':
    try:
        rospy.init_node('base_control_script', anonymous=True)

        pub = rospy.Publisher('/kmriiwa/base/command/cmd_vel', Twist, queue_size=10)

        # make sure publisher is connected
        rospy.sleep(1.0)

        speed = 0.3          # m/s
        distance = 3.0       # m
        duration = distance / speed  # 3 / 0.3 = 10s

        for i in range(100):
            rospy.loginfo(f"Cycle {i+1}/100: moving -x 3m")
            # move -x (backward if +x is forward)
            move_base(pub, -speed, 0.0, duration)

            rospy.sleep(0.5)

            rospy.loginfo(f"Cycle {i+1}/100: moving +x 3m")
            # move +x
            move_base(pub, speed, 0.0, duration)

            rospy.sleep(0.5)

        # final stop (safety)
        stop_twist = Twist()
        pub.publish(stop_twist)

    except rospy.ROSInterruptException:
        pass
