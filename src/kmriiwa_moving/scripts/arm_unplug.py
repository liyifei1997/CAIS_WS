#!/usr/bin/env python3

import rospy
from kmriiwa_msgs.msg import JointPosition
from geometry_msgs.msg import Twist
import threading
import time

def publish_arm_commands(arm_pub, joint_pos):
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        joint_pos.header.stamp = rospy.Time.now()
        arm_pub.publish(joint_pos)
        rate.sleep()



def main():
    rospy.init_node('robot_control', anonymous=True)

    arm_pub = rospy.Publisher('/kmriiwa/arm/command/JointPosition', JointPosition, queue_size=10)

    # Create JointPosition message
    joint_pos = JointPosition()
    joint_pos.header.seq = 0
    joint_pos.header.stamp = rospy.Time.now()
    joint_pos.header.frame_id = ''

    # Set initial joint positions (update these values for the specific task)
    joint_pos.a1 = 0.0  
    joint_pos.a2 = 0.0  
    joint_pos.a3 = 0.0  
    joint_pos.a4 = 0.0  
    joint_pos.a5 = 0.0  
    joint_pos.a6 = 0.0  
    joint_pos.a7 = 0.0  

    # Create threads for publishing commands
    arm_thread = threading.Thread(target=publish_arm_commands, args=(arm_pub, joint_pos))

    arm_thread.start()

    # Allow some time for the arm to reach the initial position
    time.sleep(5)

    # need to pdate joint angles based on actual solution for unplugging the charger
    joint_pos.a1 = 0.1 
    joint_pos.a2 = 0.3  
    joint_pos.a3 = -0.4  
    joint_pos.a4 = 0.1  
    joint_pos.a5 = 0.0  
    joint_pos.a6 = 0.5  
    joint_pos.a7 = -0.6  

    # Allow time for the arm to move
    time.sleep(3)

    # Add logic to close the gripper (if applicable)
    joint_pos.a1 = 0.1 
    joint_pos.a2 = 0.3  
    joint_pos.a3 = -0.4  
    joint_pos.a4 = 0.1  
    joint_pos.a5 = 0.0  
    joint_pos.a6 = 0.5  
    joint_pos.a7 = 0.6 

    # Allow some time to hold the plug
    time.sleep(2)

    # Move the arm back to a neutral position after unplugging
    joint_pos.a1 = 0.0
    joint_pos.a2 = 0.0
    joint_pos.a3 = 0.0
    joint_pos.a4 = 0.0
    joint_pos.a5 = 0.0
    joint_pos.a6 = 0.0
    joint_pos.a7 = 0.0

    # Allow some time to return to the neutral position
    time.sleep(3)

    # Start threads for publishing
    arm_thread.join()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
