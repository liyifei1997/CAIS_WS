#!/usr/bin/env python3

import rospy
from kmriiwa_msgs.msg import JointPosition
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

# Global variable for radar reading
latest_radar_distance = float('inf')

def radar_callback(msg):
    global latest_radar_distance
    latest_radar_distance = msg.data

def move_base_until_distance(linear_x, linear_y, stop_distance=0.5):
    pub = rospy.Publisher('/kmriiwa/base/command/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/radar/distance', Float32, radar_callback)

    rospy.sleep(1)
    rate = rospy.Rate(50)
    twist = Twist()
    twist.linear.x = linear_x
    twist.linear.y = linear_y

    rospy.loginfo("Starting base movement until radar < %.2f m", stop_distance)

    while not rospy.is_shutdown() and latest_radar_distance > stop_distance:
        pub.publish(twist)
        rospy.loginfo("Radar distance: %.2f m", latest_radar_distance)
        rate.sleep()

    # Stop base
    twist.linear.x = 0
    twist.linear.y = 0
    pub.publish(twist)
    rospy.loginfo("Base stopped at %.2f m", latest_radar_distance)

def move_arm(joint_positions):
    arm_pub = rospy.Publisher('/kmriiwa/arm/command/JointPosition', JointPosition, queue_size=10)
    rospy.sleep(1)
    
    for pos in joint_positions:
        pos.header.stamp = rospy.Time.now()
        arm_pub.publish(pos)
        rospy.loginfo("Published joint pos: %s", pos)
        rospy.sleep(5)

def main():
    rospy.init_node('kmr_task_radar', anonymous=True)

    # Joint positions
    original_pos = JointPosition(a1=0.0, a2=0.0, a3=0.0, a4=0.0, a5=0.0, a6=0.0, a7=0.0)
    reach_pos = JointPosition(a1=0, a2=1.9, a3=1, a4=0, a5=0, a6=0, a7=0)
    grab_pos = JointPosition(a1=0, a2=1.9, a3=0, a4=1.5, a5=0, a6=0, a7=0)
    place_pos = JointPosition(a1=0, a2=1.9, a3=1, a4=0, a5=0, a6=0, a7=0)

    # Move to Table A using radar
    move_base_until_distance(linear_x=0.2, linear_y=0.0, stop_distance=0.5)
    rospy.sleep(2)

    # Pick up item
    move_arm([original_pos, reach_pos])
    move_arm([reach_pos, grab_pos])
    
    # Move to Table B (backward using radar)
    move_base_until_distance(linear_x=-0.2, linear_y=0.0, stop_distance=0.5)
    
    # Place item
    move_arm([place_pos, original_pos])

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down KMR iiwa radar task")

'''

import rospy
from kmriiwa_msgs.msg import JointPosition


def main():
    # Initialize the ROS node
    rospy.init_node('robot_control', anonymous=True)

    # Publisher for arm joint positions
    arm_pub = rospy.Publisher('/kmriiwa/arm/command/JointPosition', JointPosition, queue_size=10)

    # Define the rate at which to publish (10 Hz)/ (30 Hz)/ (50 Hz)/ (100 Hz)
    rate = rospy.Rate(10)

    # Define the different joint positions (original, position 1, position 2)
    original_pos = JointPosition()
    original_pos.header.seq = 0
    original_pos.header.stamp = rospy.Time.now()
    original_pos.header.frame_id = ''
    original_pos.a1 = 0.0
    original_pos.a2 = 0.0
    original_pos.a3 = 0.0
    original_pos.a4 = 0.0
    original_pos.a5 = 0.0
    original_pos.a6 = 0.0
    original_pos.a7 = 0.0

    pos1 = JointPosition()
    pos1.header.seq = 0
    pos1.header.stamp = rospy.Time.now()
    pos1.header.frame_id = ''
    pos1.a1 = 0
    pos1.a2 = 1.9
    pos1.a3 = 0
    pos1.a4 = 1.5
    pos1.a5 = 0
    pos1.a6 = 0
    pos1.a7 = 0

    pos2 = JointPosition()
    pos2.header.seq = 0
    pos2.header.stamp = rospy.Time.now()
    pos2.header.frame_id = ''
    pos2.a1 = 0
    pos2.a2 = 1.9
    pos2.a3 = 1
    pos2.a4 = 0
    pos2.a5 = 0
    pos2.a6 = 0
    pos2.a7 = 0

    pos3 = JointPosition()
    pos3.header.seq = 0
    pos3.header.stamp = rospy.Time.now()
    pos3.header.frame_id = ''
    pos3.a1 = 0
    pos3.a2 = 1.9
    pos3.a3 = 1
    pos3.a4 = 0
    pos3.a5 = 0
    pos3.a6 = 0
    pos3.a7 = 0

    pos4 = JointPosition()
    pos4.header.seq = 0
    pos4.header.stamp = rospy.Time.now()
    pos4.header.frame_id = ''
    pos4.a1 = -2.6
    pos4.a2 = 0
    pos4.a3 = 0
    pos4.a4 = 1.65
    pos4.a5 = 0.5
    pos4.a6 = 0
    pos4.a7 = 0

    # Create a list of positions to move through
    positions = [original_pos, pos1, pos2, pos3, original_pos]

    # Loop through the positions
  
    for pos in positions:
        # Update the timestamp for each position
        pos.header.stamp = rospy.Time.now()

        # Publish the current joint position
        arm_pub.publish(pos)

        # Log the published message
        rospy.loginfo("Published arm joint positions: %s", pos)

        # Sleep to maintain the loop rate
        # sleep so the arm gets enough time to the current posiition before recieving the next instrctuion
        rospy.sleep(19)
        rate.sleep()
        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
        '''
