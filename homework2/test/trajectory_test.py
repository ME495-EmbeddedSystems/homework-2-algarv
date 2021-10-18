#!/usr/bin/env python

import unittest
import sympy as sym
from homework2.calc_trajectory import trajectory

class TestCase(unittest.TestCase):
    def test_x(self):
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
    
    def test_y(self):
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

    def test_linear_velocity(self):
        W = 0
        H = 0
        T = 1
        self.t = sym.symbols('t')
        self.traj = trajectory(W,H,T)
        self.assertEquals(self.traj.linear_velocity(0),0)
        
    def test_angular_velocity(self):
        W = 0
        H = 0
        T = 1
        self.t = sym.symbols('t')
        self.traj = trajectory(W,H,T)
        self.assertEquals(self.traj.angular_velocity(0),0)


if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(homework2, 'Trajectory_Test', TestCase)


