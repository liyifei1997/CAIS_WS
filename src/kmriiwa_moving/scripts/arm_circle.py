#!/usr/bin/env python3

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
    
    t1 = np.arctan2(-yc, -xc)
    th1 = np.degrees(t1)
    
    if th1 < -170 or th1 > 170:
        raise ValueError("Out of workspace")
    
    A = (xc**2) + (yc**2) + (zcc**2)
    B = -zcc * ((xc**2) + (yc**2) + (zcc**2) + (d1**2) - (d2**2)) / d1
    C = zcc**2 - (d2**2) * (1 - c4**2)
    
    Del = np.sqrt(B**2 - 4 * A * C)
    c2a = (-B + Del) / (2 * A)
    c2b = (-B - Del) / (2 * A)
    
    t2a = np.arccos(c2a) if -1 <= c2a <= 1 else 0
    t2b = np.arccos(c2b) if -1 <= c2b <= 1 else 0
    
def solve_angles(t1, t2, c4, Rd):
    solutions = []
    for temp2 in [t2, -t2]:
        th2 = np.degrees(temp2)
        temp4 = np.arccos(c4)
        th4 = np.degrees(temp4)
        Tb4o = np.array([
            d1 * np.cos(t1) * np.sin(temp2) - d2 * np.cos(t1) * np.sin(temp4 - temp2),
            d1 * np.sin(t1) * np.sin(temp2) - d2 * np.sin(t1) * np.sin(temp4 - temp2),
            db + d2 * np.cos(temp4 - temp2) + d1 * np.cos(temp2)
        ])
        
        if np.all(np.abs(Tb4o - Oc) < 0.01):
            Rb4T = np.array([
                [np.cos(temp2 - temp4) * np.cos(t1), np.cos(temp2 - temp4) * np.sin(t1), -np.sin(temp2 - temp4)],
                [-np.sin(t1), np.cos(t1), 0],
                [np.sin(temp2 - temp4) * np.cos(t1), np.sin(temp2 - temp4) * np.sin(t1), np.cos(temp2 - temp4)]
            ])
            Rwrist = Rb4T @ Rd
            temp5 = np.arctan2(Rwrist[1, 2], Rwrist[0, 2])
            temp6 = np.arccos(Rwrist[2, 2])
            temp7 = np.arctan2(Rwrist[2, 1], -Rwrist[2, 0])
            
            th5 = np.degrees(temp5)
            th6 = np.degrees(temp6)
            th7 = np.degrees(temp7)
            
            if (-170 <= th5 <= 170) and (-120 <= th6 <= 120) and (-175 <= th7 <= 175):
                solutions.append([np.degrees(t1), np.degrees(temp2), 0, np.degrees(temp4), np.degrees(temp5), np.degrees(temp6), np.degrees(temp7)])
    
    return solutions

    solutions = []
    solutions.extend(solve_angles(t1, t2a, c4, Rd))
    solutions.extend(solve_angles(t1, t2b, c4, Rd))
    
    if not solutions:
        raise ValueError("No solutions found, out of workspace")
    
    return solutions

def move_arm(joint_positions):
    pub = rospy.Publisher('/kmriiwa/arm/command/JointPosition', JointPosition, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    joint_position = JointPosition()
    joint_position.a1, joint_position.a2, joint_position.a3, joint_position.a4, joint_position.a5, joint_position.a6, joint_position.a7 = joint_positions

    pub.publish(joint_position)
    rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('circular_path_control_script', anonymous=True)

        # Define the parameters for the circular path
        radius = 0.1  # meters
        center_x = 0.5
        center_y = 0.5
        center_z = 0.5
        num_points = 100
        angular_speed = 0.1  # radians per second

        # Calculate the time to complete one full circle
        circle_duration = 2 * math.pi / angular_speed

        # Define the end effector orientation Rd
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(np.radians(180)), -np.sin(np.radians(180))],
            [0, np.sin(np.radians(180)), np.cos(np.radians(180))]
        ])
        Ry = np.array([
            [np.cos(np.radians(45)), 0, np.sin(np.radians(45))],
            [0, 1, 0],
            [-np.sin(np.radians(45)), 0, np.cos(np.radians(45))]
        ])
        Rd = Rx @ Ry

        # Calculate the positions along the circular path
        for i in range(num_points):
            theta = 2 * math.pi * (i / num_points)
            x = center_x + radius * math.cos(theta)
            y = center_y + radius * math.sin(theta)
            z = center_z

            try:
                joint_positions = inverse_kinematics(x, y, z, Rd)
                # Choose the first solution (or implement a better selection criteria)
                move_arm(joint_positions[0])
            except ValueError as e:
                rospy.loginfo(f"No valid joint positions found for point ({x}, {y}, {z})")

            time.sleep(circle_duration / num_points)

    except rospy.ROSInterruptException:
        pass
