# -*- coding: utf-8 -*-
"""
Created on Fri Dec 20 12:25:06 2019

@author: abhij
"""

def circle_intersection(f1 = 0, f2 = 0, o1 = 0, o2 = 0, l1 = 0, l2 = 0):
    R = (f1**2 + f2**2)**(0.5)
    
    x_cor = f1 - o1
    y_cor = f2 - o2
    
    
    rot1 = (l1**2 - l2**2 + R**2)/(2*R)
    print("\n l is", rot1)
    rot2 = (l1**2 - rot1**2)**0.5
    print("\n h is", rot2)
    
    x = ((rot1/R) * (x_cor)) + ((rot2/R) * (y_cor)) + o1
    y = ((rot1/R) * (y_cor)) - ((rot2/R) * (x_cor)) + o2
    
    print("the x coordinate is:", x, "\n")
    print("the y coordinate is:", y, "\n")
    
circle_intersection(2,3,0,0,4,5)