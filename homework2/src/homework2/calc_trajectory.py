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

        x = (W/2)*sin((2*pi*self.t)/T)
        y = (H/2)*sin((4*pi*self.t)/T)
        
        self.x_dot = x.diff(self.t)
        self.y_dot = y.diff(self.t)
    
    def linear_velocity(self,time):
        v_eq = sqrt(self.x_dot**2 + self.y_dot**2) 
        v_func = sym.lambdify(self.t,v_eq,modules=sym)
        self.v = v_func(time)
        return self.v
    
    def angular_velocity(self,time):
        theta = sym.atan(self.y_dot/self.x_dot)
        w_eq = theta.diff(self.t)
        w_func = sym.lambdify(self.t,w_eq,modules=sym)
        self.w = w_func(time)
        return self.w

