#!/usr/bin/env python

##CALC_TRAJECTORY MODULE##
'''
The calc_trajectory python module defines a class that recieves width, height, and period parameters and calculates
functions for the linear and angular velocities in terms of time. 

For testing purposes, the intermediate steps to calculate the x and y positions and their first and second derivatives 
are included in separate functions that can return the intermediate calculate values also in terms of time. 

Finally, another function was included to calculate theta at time 0 to simplify the static transform in the trajectory node. 
'''

import sympy as sym
import numpy as np
from numpy import *
from sympy import *

class trajectory():
    def __init__(self,W,H,T):
        #Define variables for width, height, period, and time
        self.W = W
        self.H = H
        self.T = T 
        self.t = symbols('t')

    def calc_x(self):
        #Derive an expression for the x position and its first and second derivatives with respect to t with the given function
        #Return the expressions for x, the derivative of x, and the second derivative of x
        self.x = (self.W/2)*sin((2*pi*self.t)/self.T)
        self.x_dot = self.x.diff(self.t)
        self.x_ddot = self.x_dot.diff(self.t)

        return self.x, self.x_dot, self.x_ddot
        
    def calc_y(self):
        #Derive an expression for the y position and its first and second derivatives with respect to t with the given function
        #Return the expressions for y, the derivative of y, and the second derivative of y
        self.y = (self.H/2)*sin((4*pi*self.t)/self.T)
        self.y_dot = self.y.diff(self.t)
        self.y_ddot = self.y_dot.diff(self.t)

        return self.y, self.y_dot, self.y_ddot

    def theta0(self):
        #Calculate theta at t = 0 and return theta as a float value
        self.x, self.x_dot, self.x_ddot = trajectory(self.W,self.H,self.T).calc_x()
        self.y, self.y_dot, self.y_ddot = trajectory(self.W,self.H,self.T).calc_y()
        theta0_eq = atan(self.y_dot / self.x_dot)
        theta0_func = sym.lambdify(self.t,theta0_eq,modules=sym)
        theta0 = theta0_func(0)
        return theta0

    def linear_velocity(self,time):
        #Derive an expression for linear velocity in terms of time, and solve with the given time argument
        #Return a single float value for linear velocity.
        self.x, self.x_dot, self.x_ddot = trajectory(self.W,self.H,self.T).calc_x()
        self.y, self.y_dot, self.y_ddot = trajectory(self.W,self.H,self.T).calc_y()
        v_eq = sqrt(self.x_dot**2 + self.y_dot**2) 
        v_func = sym.lambdify(self.t,v_eq,modules=sym)
        self.v = v_func(time)
        return self.v
    
    def angular_velocity(self,time):
        #Derive an expression for angular velocity in terms of time, and solve with the given time argument
        #Return a single float value for angular velocity.
        self.x, self.x_dot, self.x_ddot = trajectory(self.W,self.H,self.T).calc_x()
        self.y, self.y_dot, self.y_ddot = trajectory(self.W,self.H,self.T).calc_y()
        theta = sym.atan(self.y_dot/self.x_dot)
        w_eq = theta.diff(self.t)
        w_func = sym.lambdify(self.t,w_eq,modules=sym)
        self.w = w_func(time)
        return self.w

