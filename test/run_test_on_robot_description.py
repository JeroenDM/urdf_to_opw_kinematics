#!/usr/bin/env python
import sys
import unittest
import rostest
from urdf_parser_py.urdf import URDF
from urdf_to_opw_kinematics.main import convert

PI = 3.14159265359

# the key in the test_data dict must be the same as the robot name
# in the urdf file available on the parameter server
test_data = {}
test_data["robot1"] = {
    "c1":  0.100,
    "c2":  0.200,
    "c3":  0.100,
    "c4":  0.050,
    "a1":  0.200,
    "a2": -0.050,
    "b":   0.000,
    "joint_offsets": [0, 0, 0, 0, 0, 0],
    "sign_corrections": [1, 1, 1, 1, 1, 1]
}

test_data["irb2400"] = {
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

class TestRobot1(unittest.TestCase):

    def setUp(self):
        pass

    def test_convert(self):
        robot = URDF.from_parameter_server()
        print("Testing robot with name: " + robot.name)
        actual = convert(robot)
        if not test_data.has_key(robot.name):
            raise KeyError("Robot " + robot.name + " not found in testdata")
        expected = test_data[robot.name]
        self.compare(actual, expected)

    def compare(self, expected, actual):
        for key in actual:
            if type(actual[key]) == list:
                self.assertEqual( len(actual[key]), len(expected[key]) )
                for i in range(len(actual[key])):
                   self.assertAlmostEqual(actual[key][i], expected[key][i], places=5) 
            else:
                self.assertAlmostEqual(actual[key], expected[key], places=5)

if __name__ == '__main__':
    rostest.rosrun('urdf_to_opw_kinematics', 'run_test_on_robot_description', TestRobot1, sys.argv)
    #unittest.main()