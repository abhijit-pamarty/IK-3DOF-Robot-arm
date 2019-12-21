# -*- coding: utf-8 -*-
"""
Created on Fri Dec 20 12:25:06 2019

@author: abhij
"""

import numpy as num
import matplotlib.pyplot as plt

#_______________________________________________________________________________________________________________________#
# Function to find the intersection of two circles, returns an array with two arrays having x amd y
    
def circle_intersection(f1 = 0, f2 = 0, o1 = 0, o2 = 0, l1 = 0, l2 = 0): 
    R = (f1**2 + f2**2)**(0.5)
    
    x_cor = f1 - o1
    y_cor = f2 - o2
    
    
    rot1 = (l1**2 - l2**2 + R**2)/(2*R)
    rot2 = (l1**2 - rot1**2)**0.5
    
    x = ((rot1/R) * (x_cor)) + ((rot2/R) * (y_cor)) + o1
    y = ((rot1/R) * (y_cor)) - ((rot2/R) * (x_cor)) + o2
    
    value1 = []
    value1.append(x)
    value1.append(y)
    
    x = ((rot1/R) * (x_cor)) - ((rot2/R) * (y_cor)) + o1
    y = ((rot1/R) * (y_cor)) + ((rot2/R) * (x_cor)) + o2

    value2 = []
    value2.append(x)
    value2.append(y)
    
    values = []
    
    values.append(value1)
    values.append(value2)
    
    return values
#_______________________________________________________________________________________________________________________#
# Function to find angle between two lines
    
def generate_line(point_1 , point_2 ):
    if (point_2[0] - point_1[0]) != 0:
        slope = (point_2[1] - point_1[1])/(point_2[0] - point_1[0])
    else:
        slope = 1000000 #effectively infinity
    constant = -1*slope*point_1[0] + point_1[1]
    line = []
    line.append(slope)
    line.append(constant)
    return line
#_______________________________________________________________________________________________________________________#
# Function to find angle between two lines
def ang_between_lines(line1, line2):
    return(num.arctan(abs((line1[0] - line2[0])/(1 + (line1[0]*line2[0])))))
    
#_______________________________________________________________________________________________________________________#
# Function to find the angle of the servos 
def extension(X_cord, Y_cord, target_X, target_Y ,f_length, b_length):
    
    values = circle_intersection(target_X, target_Y, 0, 0, f_length, b_length)
    
    optimal_point = []
    min_distance = 10000000
    print("points")
    for value in values:
        print(value)
        distance = (((value[0] - X_cord)**2)+((value[1] - Y_cord)**2))**0.5
        if distance <= min_distance:
            optimal_point = value
        
    print("optimal point:\n", optimal_point)
    origin = [0,0]
    target_point = [target_X, target_Y]
    
    
    plt.scatter(0,0, color = "white") #to define the boundaries of the graph
    plt.scatter(10,10, color = "white")
    plt.scatter(0,10, color = "white")
    plt.scatter(10,0, color = "white")
    
    x_values_bicep = [origin[0],optimal_point[0]]
    y_values_bicep = [origin[1],optimal_point[1]]
    
    plt.plot(x_values_bicep,y_values_bicep, color = "black", linewidth = 5)
    
    x_values_forearm = [optimal_point[0],target_point[0]]
    y_values_forearm = [optimal_point[1],target_point[1]]
    
    plt.plot(x_values_forearm,y_values_forearm, color = "black", linewidth = 5)
    
    
    plt.scatter(origin[0],origin[1], color = "black", s = 250,  marker = "s")
    plt.scatter(target_point[0],target_point[1], color = "red", s = 200, marker = 9)
    plt.scatter( optimal_point[0], optimal_point[1], color = "red", s= 100, marker = "s")
    plt.show()
    
    bicep_line = generate_line(origin, optimal_point)
    forearm_line = generate_line(optimal_point, target_point)
    
    x_axis = [0,0]
    
    angles = []

    shoulder_angle = ang_between_lines(x_axis,bicep_line)
    elbow_angle = ang_between_lines(forearm_line, bicep_line)
    
    angles.append(shoulder_angle)
    angles.append(elbow_angle)
    
    return(angles)
        
        
    

#_______________________________________________________________________________________________________________________#
# Main Function

def Inverse_kinematics(final_x, final_y, final_z, init_x, init_y, init_z):
    
    
    check_bounds = (final_x**2) + (final_y**2) + (final_z**2)
    check_floor  = bool(final_z > 0)
    
    base_size = 0.5
    f_length = 5
    b_length = 5
    
    reach = (f_length + b_length)**2
    
    if check_bounds > base_size and check_bounds < reach and check_floor and final_y >= 0:
    
        
        base_x = 0
        base_y = 0
        base_z = 0
        angles = []
        
        R_init = (((init_x - base_x)**2)+((init_y - base_y)**2))**0.5
        R_fin = (((final_x - base_x)**2)+((final_y - base_y)**2))**0.5
        theta = num.arctan((final_y - base_y)/(final_x - base_x))
        Z_init = init_z - base_z
        Z_fin = final_z - base_z
        
        
        angles = extension(R_init, Z_init, R_fin, Z_fin, f_length, b_length)
        
        servo_angle_1 = (theta/3.14)*180 
        servo_angle_2 = (angles[0]/3.14)*180 
        servo_angle_3 = (angles[1]/3.14)*180 
        print("servo 1's angle is", servo_angle_1)
        print("servo 2's angle is", servo_angle_2)
        print("servo 3's angle is", servo_angle_3)
    
    elif check_bounds < base_size:
        print("Action terminated, collision with base")
    
    elif check_bounds > reach or final_y < 0:
        print("Action terminated, not enough reach")
    
    elif check_floor == 0:
        print("Action terminated, collision with floor")
    else:
        print("Unknown error, please terminate the program")
  
Inverse_kinematics(7,0,1,6,0,1)
