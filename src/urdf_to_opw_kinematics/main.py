
#!/usr/bin/env python
import numpy as np
from numpy.linalg import norm
from urdf_to_opw_kinematics.util import angle, Axis

def convert(robot):
    axes = get_joint_axes_from_urdf(robot)
    tool0_position = get_tool0_position(robot, axes)


    jo = get_joint_offsets(axes)

    sc = get_sign_corrections(axes)

    params = get_dimensions(axes, tool0_position)
    #params = get_dimensions_new(axes)
    params['joint_offsets'] = jo
    params['sign_corrections'] = sc
    return params

def get_joint_axes_from_urdf(robot):
    joints = robot.joints
    axes = []
    for i in range(len(joints)):
        if joints[i].type == "revolute":
            if i > 0:
                p_relative = np.array(joints[i].origin.xyz)
                p_previous = axes[-1].p
                axes.append(Axis( p_previous + p_relative, p_relative, np.array(joints[i].axis) ))
            else:
                axes.append(Axis( np.array(joints[i].origin.xyz), np.array(joints[i].origin.xyz), np.array(joints[i].axis) ))     
    return axes

def get_tool0_position(robot, axes):
    for joint in robot.joints:
        if joint.child == "tool0":
            return axes[-1].p + np.array(joint.origin.xyz)
    raise ValueError("Failed to find a joint with child link 'tool0'.")

def get_joint_offsets(axes):

    v23 = axes[1].shortest_distance_vector(axes[2])
    unit_z = np.array([0, 0, 1.0])
    jo2 = -angle(unit_z, v23)

    g4_positive = np.array([abs(e) for e in axes[3].d])
    jo3 = -angle(v23, g4_positive)

    return [0.0, jo2, jo3, 0.0, 0.0, 0.0]

def get_sign_corrections(axes):
    """ axis positive rotation convention
    Assume all z-axis pointed up and axis along one of the main axes
    """
    sc = map(np.sum, [a.d for a in axes])
    return [int(val) for val in sc]

def get_dimensions(axes, tool0_position):
    params = {}
    # check if a1 is along positive x and g2 is above x-y plane
    # this mean that the position of g2 should be (a1, 0, c1) with a1 > 0
    P2 = axes[1].p
    if (P2[0] >= 0 and P2[1] == 0 and P2[2] >= 0):
        params['a1'] = P2[0]
        params['c1'] = P2[2]
    else:
        raise ValueError("Wrong orientations of g2.")

    params['c2'] = axes[1].shortest_distance(axes[2])

    v35 = axes[2].shortest_distance_vector(axes[4])
    params['c3'] = np.abs(np.dot(v35, axes[3].d))

    v34 = axes[2].shortest_distance_vector(axes[3])
    #a2_sign = np.sign(np.dot(v34))
    a2_sign = -1
    params['a2'] = norm(v34) * a2_sign

    # distance between g3 and g4 perpendicular along g3
    
    params['b'] = np.dot(v34, axes[2].d)

    # distance between g5 and tool0 along g6
    params['c4'] = np.abs(np.dot(axes[5].d, tool0_position - axes[4].p))
    return params

def get_dimensions_new(axes):
    params = {}
    P_0_1 = axes[0].p_rel
    P_1_2 = axes[1].p_rel
    P_2_3 = axes[2].p_rel
    P_3_4 = axes[3].p_rel
    P_4_5 = axes[4].p_rel
    P_5_6 = axes[5].p_rel

    params['c1'] = P_0_1[2] + P_1_2[2]
    params['c2'] = norm(P_2_3)
    params['c3'] = norm(P_4_5) + P_3_4[0]
    params['c4'] = norm(P_5_6)

    params['a1'] = np.sqrt(P_1_2[0]**2 + P_1_2[1]**2)
    params['a2'] = -np.sqrt(P_3_4[0]**2 + P_3_4[2]**2) # or -P_3_4[2]
    params['b']  = P_3_4[1]
    return params