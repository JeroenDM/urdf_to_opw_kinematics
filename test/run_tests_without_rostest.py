#!/usr/bin/env python
##########################################################################################
# With this file you can run the tests on the three robots without using
# the rostest framework.
# This was my original test and I left it in because it is still up to date
# It can be useful if you like clean test output instead of the 'catkin run_tests' output
##########################################################################################

import unittest
from urdf_parser_py.urdf import URDF
from urdf_to_opw_kinematics.main import convert

PI = 3.14159265359;
# TODO change this so that I don't have to use absolute paths
PATH = '/home/jeroen/Dropbox/ros/opw_plugin/src/urdf_to_opw_kinematics/urdf/'

class TestRobots(unittest.TestCase):

    def setUp(self):
        self.kr6r700sixx = {
            "c1":  0.400,
            "c2":  0.315,
            "c3":  0.365,
            "c4":  0.080,
            "a1":  0.025,
            "a2": -0.035,
            "b":   0.000,
            "joint_offsets": [0, -PI/2, 0, 0, 0, 0],
            "sign_corrections": [-1, 1, 1, -1, 1, -1]
        }

        self.robot1 = {
            "c1":  0.070,
            "c2":  0.200,
            "c3":  0.100,
            "c4":  0.050,
            "a1":  0.200,
            "a2":  0.050,
            "b":   0.000,
            "joint_offsets": [0, 0, 0, 0, 0, 0],
            "sign_corrections": [1, 1, 1, 1, 1, 1]
        }

        self.irb2400 = {
            "c1":  0.615,
            "c2":  0.705,
            "c3":  0.755,
            "c4":  0.085,
            "a1":  0.100,
            "a2": -0.135,
            "b":   0.000,
            "joint_offsets": [0, 0, -PI / 2, 0, 0, 0],
            "sign_corrections": [1, 1, 1, 1, 1, 1]
        }
    
    def test_kuka_kr6r700sixx(self):
        robot = URDF.from_xml_file(PATH + 'kr6r700sixx.urdf')
        actual = convert(robot)
        self.compare(actual, self.kr6r700sixx)
    
    def test_robot1(self):
        robot = URDF.from_xml_file(PATH + 'robot1.urdf')
        actual = convert(robot)
        self.compare(actual, self.robot1)
    
    def test_abb_irb2400(self):
        robot = URDF.from_xml_file(PATH + 'irb2400.urdf')
        actual = convert(robot)
        self.compare(actual, self.irb2400)

    def compare(self, expected, actual):
        for key in actual:
            if type(actual[key]) == list:
                self.assertEqual( len(actual[key]), len(expected[key]) )
                for i in range(len(actual[key])):
                   self.assertAlmostEqual(actual[key][i], expected[key][i], places=5) 
            else:
                self.assertAlmostEqual(actual[key], expected[key], places=5)

if __name__ == '__main__':
    unittest.main()