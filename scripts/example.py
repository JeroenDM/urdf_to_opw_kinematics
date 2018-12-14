#!/usr/bin/env python
import rospy
from urdf_parser_py.urdf import URDF

from urdf_to_opw_kinematics.main import convert

PI = 3.14159265359;

params_kr6 = {
    "c1":  0.400,
    "c2":  0.315,
    "c3":  0.365,
    "c4":  0.080,
    "a1":  0.025,
    "a2": -0.035,
    "b":   0.000,
    "joint_offsets": [0, -PI / 2, 0, 0, 0, 0],
    "sign_corrections": [-1, 1, 1, -1, 1, -1]
}

def run():
    robot = URDF.from_parameter_server()
    print(robot.name == "abb_irb2400")

    params = convert(robot)

    for key in params:
        print(key + ":\t" + str(params[key]) + "\t" + str(params_kr6[key]))

if __name__ == "__main__":
    run()
   