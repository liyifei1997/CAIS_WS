
#!/usr/bin/env python3

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
    pos1.a1 = 2.9
    pos1.a2 = 0
    pos1.a3 = 0
    pos1.a4 = 1.6
    pos1.a5 = 0.5
    pos1.a6 = 0
    pos1.a7 = 0.5

    pos2 = JointPosition()
    pos2.header.seq = 0
    pos2.header.stamp = rospy.Time.now()
    pos2.header.frame_id = ''
    pos2.a1 = 1.25
    pos2.a2 = 0
    pos2.a3 = 0
    pos2.a4 = 1.65
    pos2.a5 = 1
    pos2.a6 = 0
    pos2.a7 = 0

    pos3 = JointPosition()
    pos3.header.seq = 0
    pos3.header.stamp = rospy.Time.now()
    pos3.header.frame_id = ''
    pos3.a1 = 0
    pos3.a2 = 0
    pos3.a3 = 0
    pos3.a4 = 1.6
    pos3.a5 = 1.5
    pos3.a6 = 0.1
    pos3.a7 = 0

    pos3 = JointPosition()
    pos3.header.seq = 0
    pos3.header.stamp = rospy.Time.now()
    pos3.header.frame_id = ''
    pos3.a1 = -1.25
    pos3.a2 = 0
    pos3.a3 = 0
    pos3.a4 = 1.65
    pos3.a5 = 2
    pos3.a6 = 0.2
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
    positions = [pos1, pos2, pos3, pos4, pos1]

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