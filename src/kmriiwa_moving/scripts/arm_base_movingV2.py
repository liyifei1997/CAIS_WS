#!/usr/bin/env python3
'''#!/usr/bin/env python3

import rospy
from kmriiwa_msgs.msg import JointPosition
from geometry_msgs.msg import Twist

def main():
    # Initialize the ROS node
    rospy.init_node('robot_control', anonymous=True)

    # Publishers for arm joints and mobile base
    # arm_pub = rospy.Publisher('/kmriiwa/arm/command/JointPosition', JointPosition, queue_size=10)
    base_pub = rospy.Publisher('/kmriiwa/base/command/cmd_vel', Twist, queue_size=10)

    # Define the rate at which to publish (10 Hz)
    rate = rospy.Rate(10)

    # Create JointPosition message
    # joint_pos = JointPosition()
    # joint_pos.header.seq = 0
    # joint_pos.header.stamp = rospy.Time.now()
    # joint_pos.header.frame_id = ''
    # joint_pos.a1 = 0
    # joint_pos.a2 = 0
    # joint_pos.a3 = 0
    # joint_pos.a4 = 0
    # joint_pos.a5 = 0
    # joint_pos.a6 = 0
    # joint_pos.a7 = 0

    # Create Twist message
    twist = Twist()
    twist.linear.x = -0.2
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0

    while not rospy.is_shutdown():
        # Update the timestamp
        # joint_pos.header.stamp = rospy.Time.now()

        # Publish the messages
        # arm_pub.publish(joint_pos)
        base_pub.publish(twist)

        # Sleep to maintain the loop rate
        rospy.sleep(1)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass '''
'''
#!/usr/bin/env python3

import rospy
from kmriiwa_msgs.msg import JointPosition
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



def main():
    # Initialize the ROS node
    rospy.init_node('robot_control', anonymous=True)

    # Publishers for arm joints and mobile base
    arm_pub = rospy.Publisher('/kmriiwa/arm/command/JointPosition', JointPosition, queue_size=10)
    base_pub = rospy.Publisher('/kmriiwa/base/command/cmd_vel', Twist, queue_size=10)

    # Define the rate at which to publish (10 Hz)
    rate = rospy.Rate(10)

    # Create JointPosition message
    joint_pos = JointPosition()
    joint_pos.header.seq = 0
    joint_pos.header.stamp = rospy.Time.now()
    joint_pos.header.frame_id = ''
    joint_pos.a1 = 0.6
    joint_pos.a2 = 0.5
    joint_pos.a3 = -0.5
    joint_pos.a4 = 0.2
    joint_pos.a5 = 0.0
    joint_pos.a6 = 0.5
    joint_pos.a7 = -0.6

    # Create Twist message
    twist = Twist()
    twist.linear.x = -0.3
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0

    while not rospy.is_shutdown():
        # Update the timestamp

        
        joint_pos.header.stamp = rospy.Time.now()

        # Publish the messages in quick succession
        arm_pub.publish(joint_pos)
        base_pub.publish(twist)

        # Log the published messages
        rospy.loginfo("Published arm joint positions: %s", joint_pos)
        rospy.loginfo("Published base velocities: %s", twist)

        # Sleep to maintain the loop rate
        rospy.sleep(1)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

'''
'''

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
    pos1.a4 = 1.65
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
    pos3.a4 = 1.65
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
    '''
import rospy
from kmriiwa_msgs.msg import JointPosition
from geometry_msgs.msg import Twist

# Function to move the base
def move_base(linear_x, linear_y, duration):
    pub = rospy.Publisher('/kmriiwa/base/command/cmd_vel', Twist, queue_size=10)
    rospy.sleep(1)  # Ensure ROS is ready
    rate = rospy.Rate(200)  # 20Hz control loop
    twist = Twist()
    twist.linear.x = linear_x
    twist.linear.y = linear_y

    start_time = rospy.Time.now()
    while (rospy.Time.now() - start_time).to_sec() < duration:
        rospy.loginfo("Moving base: linear_x = %f, linear_y = %f", linear_x, linear_y)
        pub.publish(twist)
        rate.sleep()

    # Stop the base
    twist.linear.x = 0
    twist.linear.y = 0
    rospy.loginfo("Stopping base.")
    pub.publish(twist)

# Function to move the arm
def move_arm():
    arm_pub = rospy.Publisher('/kmriiwa/arm/command/JointPosition', JointPosition, queue_size=10)
    rospy.sleep(1)  # Ensure ROS is ready
    rate = rospy.Rate(1)  # Control rate

    # Define joint positions
    original_pos = JointPosition(a1=0.0, a2=0.0, a3=0.0, a4=0.0, a5=0.0, a6=0.0, a7=0.0)
    pos1 = JointPosition(a1=1, a2=1, a3=1, a4=1, a5=1, a6=1, a7=1)

    positions = [original_pos, pos1, original_pos]

    for pos in positions:
        pos.header.stamp = rospy.Time.now()
        arm_pub.publish(pos)
        rospy.loginfo("Published arm joint positions: %s", pos)
        rospy.sleep(10)  # Keep arm in position for 15s

if __name__ == '__main__':
    try:
        rospy.init_node('robot_control', anonymous=True)

        # Define movement parameters
        linear_speed = -0.2  # meters per second
        distance = -1.0  # meters
        duration = abs(distance / linear_speed)  # seconds

        # Move both base and arm simultaneously without threading
        move_base(linear_speed, 0.0, duration)
        move_arm()

    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt detected, shutting down.")

