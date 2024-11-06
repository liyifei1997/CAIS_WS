#!/usr/bin/env python3
'''
import rospy
import numpy as np
import math
import time
from kmriiwa_msgs.msg import JointPosition

def inverse_kinematics(xd, yd, zd, Rd):
    d1 = 0.420
    d2 = 0.400
    db = 0.360
    dn = 0.2
    
    Od = np.array([xd, yd, zd])
    Oc = Od - dn * Rd @ np.array([0, 0, 1])
    xc, yc, zc = Oc
    zcc = zc - db
    
    c4 = ((xc**2) + (yc**2) + (zcc**2) - (d1**2) - (d2**2)) / (2 * d1 * d2)


    if not (-1 <= c4 <= 1): #new
        raise ValueError("No valid solution for inverse kinematics.")
    

    t1 = np.arctan2(-yc, -xc)
    th1 = np.degrees(t1)
    
    if th1 < -170 or th1 > 170:
        raise ValueError("Out of workspace")
    
    A = (xc**2) + (yc**2) + (zcc**2)
    B = -zcc * ((xc**2) + (yc**2) + (zcc**2) + (d1**2) - (d2**2)) / d1
    C = zcc**2 - (d2**2) * (1 - c4**2)
    
    Del = np.sqrt(B**2 - 4 * A * C)

    if Del < 0: #new
        raise ValueError("No valid configuration.")

    c2a = (-B + Del) / (2 * A)
    c2b = (-B - Del) / (2 * A)
    
    t2a = np.arccos(c2a) if -1 <= c2a <= 1 else 0
    t2b = np.arccos(c2b) if -1 <= c2b <= 1 else 0


    # new
    joint_positions = []
    if t2a is not None:
        joint_positions.append([th1, np.degrees(t2a), 0, 0, 0, 0, 0])  # Add first solution
    if t2b is not None:
        joint_positions.append([th1, np.degrees(t2b), 0, 0, 0, 0, 0])  # Add second solution

    return joint_positions if joint_positions else None  # Return None if no valid solutions



def move_arm(joint_positions):
    pub = rospy.Publisher('/kmriiwa/arm/command/JointPosition', JointPosition, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    joint_position = JointPosition()
    joint_position.a1, joint_position.a2, joint_position.a3, joint_position.a4, joint_position.a5, joint_position.a6, joint_position.a7 = joint_positions

    pub.publish(joint_position)
    rate.sleep()

def move_from_a_to_b(point_a, point_b, steps=100):
    # Define the end effector orientation Rd
    Rd = np.eye(3)  # Identity matrix as a placeholder for orientation
    
    # Interpolate between point A and point B
    for i in range(steps + 1):
        t = i / steps
        x = (1 - t) * point_a[0] + t * point_b[0]
        y = (1 - t) * point_a[1] + t * point_b[1]
        z = (1 - t) * point_a[2] + t * point_b[2]

        try:
            joint_positions = inverse_kinematics(x, y, z, Rd) 
            if joint_positions and len(joint_positions) > 0:  # Check if there are valid solutions
                move_arm(joint_positions[0])  # Move using the first solution
            else:
                rospy.loginfo(f"No valid joint positions found for point ({x}, {y}, {z})")
        except ValueError as e:
            rospy.loginfo(f"No valid joint positions found for point ({x}, {y}, {z})")

        time.sleep(0.1)  # Sleep to control speed of movement,  can be adjusted based on how fast you want the arm to move between points.
        

if __name__ == '__main__':
    try:
        rospy.init_node('linear_path_control_script', anonymous=True)

        # Define points A and B
        point_a = (0.4, 0.4, 0.4)  # Starting point coordinates
        point_b = (0.2,0.2,0.5)  # Ending point coordinates

        move_from_a_to_b(point_a, point_b)

    except rospy.ROSInterruptException:
        pass'''

import rospy
import numpy as np
import math
import time
from kmriiwa_msgs.msg import JointPosition

def inverse_kinematics_line(start, end):
    d1 = 0.420
    d2 = 0.400
    db = 0.360
    dn = 0.2

    #d1 and d2: lengths of the arm segments.
    #db: a height offset related to the arm’s base.
    #dn: the distance from the end-effector to the point where calculations are based.
    
    # Calculate direction vector and length of the line segment
    # Here, we calculate the direction vector from the start to the end position by subtracting the start coordinates from the end coordinates. 
    # The length of this vector is computed using the Euclidean norm.
    direction = np.array(end) - np.array(start)
    length = np.linalg.norm(direction)
    
    #If the start and end points are identical (length is zero), the function raises an error since no movement is needed.
    if length == 0:
        raise ValueError("Start and end points are the same.")

    # Normalize the direction vector
    # The direction vector is normalized to get a unit vector Rd, which indicates the direction of movement without affecting its magnitude.
    Rd = direction / length

    # Compute desired position
    # Od represents the desired end-effector position in 3D space.
    # Oc calculates the corrected position by adjusting Od downwards by dn, which accounts for the end-effector's offset in the direction of the z-axis.
    # zcc is the z-coordinate adjusted for the base height offset db.
    xd, yd, zd = end
    Od = np.array([xd, yd, zd])
    Oc = Od - dn * Rd @ np.array([0, 0, 1])  # Adjust for end-effector offset
    xc, yc, zc = Oc
    zcc = zc - db

    # Inverse kinematics calculations
    #T used in the cosine law to find the angle between the two segments of the arm based on the distances.
    c4 = ((xc**2) + (yc**2) + (zcc**2) - (d1**2) - (d2**2)) / (2 * d1 * d2)

    #If c4 is outside the range of [-1, 1], there is no valid configuration for the arm to reach the target, so an error is raised.
    if not (-1 <= c4 <= 1):
        raise ValueError("No valid solution for inverse kinematics.")

    #t1 computes the first joint angle (th1) using the arctan2 function, 
    # which accounts for the signs of xc and yc to determine the correct quadrant of the angle.
    t1 = np.arctan2(-yc, -xc)
    th1 = np.degrees(t1)

    #This checks if the calculated angle is within the operational limits of the robotic arm. If not, an error is raised.
    if th1 < -170 or th1 > 170:
        raise ValueError("Out of workspace")
    
    #These lines define coefficients A, B, and C for a quadratic equation derived from the kinematics of the arm.
    A = (xc**2) + (yc**2) + (zcc**2)
    B = -zcc * ((xc**2) + (yc**2) + (zcc**2) + (d1**2) - (d2**2)) / d1
    C = zcc**2 - (d2**2) * (1 - c4**2)

    #The discriminant (Del) is computed to determine if real solutions exist for the joint angles.
    Del = np.sqrt(B**2 - 4 * A * C)

    if Del < 0:
        raise ValueError("No valid configuration.")
    
    #These lines calculate two possible solutions for the second joint angle (t2).
    c2a = (-B + Del) / (2 * A)
    c2b = (-B - Del) / (2 * A)

    #joint_positions stores the possible configurations (joint angles).
    #Each configuration includes th1 and the calculated angles t2a and t2b, which are converted to degrees. The other joint angles are initialized to zero.
    joint_positions = []
    t2a = np.arccos(c2a) if -1 <= c2a <= 1 else None
    t2b = np.arccos(c2b) if -1 <= c2b <= 1 else None

    if t2a is not None:
        joint_positions.append([th1, np.degrees(t2a), 0, 0, 0, 0, 0])  # First solution
    if t2b is not None:
        joint_positions.append([th1, np.degrees(t2b), 0, 0, 0, 0, 0])  # Second solution

    return joint_positions if joint_positions else None  # Return None if no valid solutions


def move_arm(joint_positions):
    pub = rospy.Publisher('/kmriiwa/arm/command/JointPosition', JointPosition, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    joint_position = JointPosition()
    # Make sure that the JointPosition message is being populated correctly. Check that the values are being assigned correctly
    joint_position.a1, joint_position.a2, joint_position.a3, joint_position.a4, joint_position.a5, joint_position.a6, joint_position.a7 = joint_positions
    
    pub.publish(joint_position)
    rate.sleep()

def interpolate_joint_positions(start_positions, end_positions, steps):
    '''steps: The number of interpolation steps between the start and end positions.
    The for loop iterates from 0 to steps (inclusive), which allows for a total of steps + 1 positions (including the start and end).
    t exist from 0 to 1 . At step = 0, t = 0, and at step = steps, t = 1.
    t determines how far along the transition you are between the starting and ending positions.'''
    for step in range(steps + 1):
        t = step / steps
        intermediate_positions = [
            start + t * (end - start)
            for start, end in zip(start_positions, end_positions)
        ]
        move_arm(intermediate_positions)
        time.sleep(0.1)

def move_from_a_to_b(point_a, point_b):
    # Compute end-effector positions
    Rd = np.eye(3)  # Assuming no rotation for simplicity; adjust as needed
    joint_positions_a = inverse_kinematics_line(point_a, point_a)  # Correct call
    joint_positions_b = inverse_kinematics_line(point_a, point_b)  # Use point_a as the start point


    if joint_positions_a is None or joint_positions_b is None:
        raise ValueError("Could not compute inverse kinematics for one or both points.")

    # Assuming you want to take the first valid configuration for both points
    start_positions = joint_positions_a[0]
    end_positions = joint_positions_b[0]

    # Number of interpolation steps
    steps = 100
    # make sure the start and end position 
    interpolate_joint_positions(start_positions, end_positions, steps)

if __name__ == '__main__':
    try:
        rospy.init_node('move_arm_from_a_to_b', anonymous=True)

        # Define the end-effector positions for point A and point B
        point_a = [0.0, 0.0, 0.0]  # Example coordinates in meters
        point_b = [0.2, 0.2, 0.5]  # Example coordinates in meters

        # Move the arm from point A to point B
        move_from_a_to_b(point_a, point_b)

    except rospy.ROSInterruptException:
        pass


'''def interpolate_joint_positions(start_positions, end_positions, steps):
    #t is a factor that ranges from 0 to 1.
    #At step = 0, t is 0 (corresponding to the start position).
    #At step = steps, t is 1 (corresponding to the end position).
    for step in range(steps + 1):
        t = step / steps
        intermediate_positions = [
            start + t * (end - start)
            for start, end in zip(start_positions, end_positions)
        ]
        move_arm(intermediate_positions)
        time.sleep(0.1)  # Adjust the delay as needed

if __name__ == '__main__':
    try:
        rospy.init_node('move_arm_from_a_to_b', anonymous=True)

        # Define joint positions for point A and point B
        point_a = [0, 0, 0, 0, 0, 0, 0]  # All joints at zero position
        point_b = [30, 45, 30, 90, 0, 0, 0]  #   # Example joint angles in degrees

        # Number of interpolation steps
        steps = 100

        # Move the arm from point A to point B
        interpolate_joint_positions(point_a, point_b, steps)

    except rospy.ROSInterruptException:
        pass'''