#!/usr/bin/env python

import unittest
import sympy as sym
import numpy as np
from numpy import round 
#I know that this could look like "cheating" the tests, but round is necessary for tests 
# involving calculations with pi where decimals are rounded off and the test case is an integer. 
# For example x(1) is on the order of 10^-16 which should pass the test of matching 0.
from homework2.calc_trajectory import trajectory

class TestCase(unittest.TestCase):
    def test_x(self):

        #Test at time = 0 and W = H = 0
        W = 0
        H = 0
        T = 1
        self.t = sym.symbols('t')
        self.traj = trajectory(W,H,T)
        self.x, self.x_dot, self.x_ddot = self.traj.calc_x()

        x = sym.lambdify(self.t,self.x)
        self.assertEquals(x(0),0)

        x_dot = sym.lambdify(self.t,self.x_dot)
        self.assertEquals(x_dot(0),0)

        x_ddot = sym.lambdify(self.t,self.x_ddot)
        self.assertEquals(x_dot(0),0)

        #Test at time = 1 and W = H = 2
        W = 2
        H = 2
        T = 1
        self.t = sym.symbols('t')
        self.traj = trajectory(W,H,T)
        self.x, self.x_dot, self.x_ddot = self.traj.calc_x()
    
        x = sym.lambdify(self.t,self.x)
        self.assertEquals(round(x(1)),0)

        x_dot = sym.lambdify(self.t,self.x_dot)
        self.assertEquals(round(x_dot(1)),round(2*np.pi))

        x_ddot = sym.lambdify(self.t,self.x_ddot)
        self.assertEquals(round(x_ddot(1)),0)

    def test_y(self):
    
        #Test at time = 0 and W = H = 0
        W = 0
        H = 0
        T = 1
        self.t = sym.symbols('t')
        self.traj = trajectory(W,H,T)
        self.y, self.y_dot, self.y_ddot = self.traj.calc_y()

        y = sym.lambdify(self.t,self.y)
        self.assertEquals(y(0),0)

        y_dot = sym.lambdify(self.t,self.y_dot)
        self.assertEquals(y_dot(0),0)

        y_ddot = sym.lambdify(self.t,self.y_ddot)
        self.assertEquals(y_ddot(0),0)

        #Test at time = 1 and W = H = 2
        W = 2
        H = 2
        T = 1
        self.t = sym.symbols('t')
        self.traj = trajectory(W,H,T)
        self.y, self.y_dot, self.y_ddot = self.traj.calc_y()

        y = sym.lambdify(self.t,self.y)
        self.assertEquals(round(y(1)),0)

        y_dot = sym.lambdify(self.t,self.y_dot)
        self.assertEquals(round(y_dot(1)),round(4*np.pi))

        y_ddot = sym.lambdify(self.t,self.y_ddot)
        self.assertEquals(round(y_ddot(1)),0)

    def test_linear_velocity(self):
        #Test at time = 0 and W = H = 0
        W = 0
        H = 0
        T = 1
        self.t = sym.symbols('t')
        self.traj = trajectory(W,H,T)
        self.assertEquals(self.traj.linear_velocity(0),0)

        #Test at time = 1 and W = H = 2
        W = 2
        H = 2
        T = 1
        self.t = sym.symbols('t')
        self.traj = trajectory(W,H,T)
        self.assertEquals(self.traj.linear_velocity(1),np.sqrt(20)*np.pi)
        
    def test_angular_velocity(self):
        #Test at time = 0 and W = H = 0
        W = 0
        H = 0
        T = 1
        self.t = sym.symbols('t')
        self.traj = trajectory(W,H,T)
        self.assertEquals(self.traj.angular_velocity(0),0)
    
        #Test at time = 1 and W = H = 2
        W = 2
        H = 2
        T = 1
        self.t = sym.symbols('t')
        self.traj = trajectory(W,H,T)
        self.assertEquals(self.traj.angular_velocity(0),0)


if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(homework2, 'Trajectory_Test', TestCase)


