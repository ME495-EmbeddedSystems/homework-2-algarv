#!/usr/bin/env python

import unittest
from homework2.calc_trajectory import trajectory

class TestCase(unittest.TestCase):
    def test_something(self):
        traj = trajectory(0,0,1)
        self.assertEquals(traj.linear_velocity(0),0)

if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(homework2, 'Trajectory_Test', TestCase)


