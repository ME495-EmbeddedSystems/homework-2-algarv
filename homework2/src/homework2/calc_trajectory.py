#!/usr/bin/env python

import sympy as sym
import numpy as np
from numpy import *
from sympy import *

class trajectory():

    def __init__(self,W,H,T):
        self.W = W
        self.H = H
        self.T = T 
        self.t = symbols('t')

    def calc_x(self):
        self.x = (self.W/2)*sin((2*pi*self.t)/self.T)
        self.x_dot = self.x.diff(self.t)
        self.x_ddot = self.x_dot.diff(self.t)

        return self.x, self.x_dot, self.x_ddot
        
    def calc_y(self):
        self.y = (self.H/2)*sin((4*pi*self.t)/self.T)
        self.y_dot = self.y.diff(self.t)
        self.y_ddot = self.y_dot.diff(self.t)

        return self.y, self.y_dot, self.y_ddot

    def theta0(self):
        self.x, self.x_dot, self.x_ddot = trajectory(self.W,self.H,self.T).calc_x()
        self.y, self.y_dot, self.y_ddot = trajectory(self.W,self.H,self.T).calc_y()
        theta0_eq = sqrt(self.x_dot**2 + self.y_dot**2)
        theta0_func = sym.lambdify(self.t,theta0_eq,modules=sym)
        theta0 = theta0_func(0)
        return theta0

    def linear_velocity(self,time):
        self.x, self.x_dot, self.x_ddot = trajectory(self.W,self.H,self.T).calc_x()
        self.y, self.y_dot, self.y_ddot = trajectory(self.W,self.H,self.T).calc_y()
        v_eq = sqrt(self.x_dot**2 + self.y_dot**2) 
        v_func = sym.lambdify(self.t,v_eq,modules=sym)
        self.v = v_func(time)
        return self.v
    
    def angular_velocity(self,time):
        self.x, self.x_dot, self.x_ddot = trajectory(self.W,self.H,self.T).calc_x()
        self.y, self.y_dot, self.y_ddot = trajectory(self.W,self.H,self.T).calc_y()
        theta = sym.atan(self.y_dot/self.x_dot)
        w_eq = theta.diff(self.t)
        w_func = sym.lambdify(self.t,w_eq,modules=sym)
        self.w = w_func(time)
        return self.w

