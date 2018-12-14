#!/usr/bin/env python
import numpy as np
from numpy.linalg import norm

TOL = 1e-6

def angle(v1, v2):
    cos = np.dot(v1, v2) / (norm(v1) * norm(v2))
    sin = norm(np.cross(v1, v2))
    return np.arctan2(sin, cos)

class Axis:
    def __init__(self, position, position_rel, direction):
        self.position = position
        self.p_rel = position_rel
        self.direction = direction / norm(direction)
    
    def __str__(self):
        s = "[Axis]\tpoint: "
        s += str(self.position) + "\tdir: "
        s += str(self.direction) + "\n"
        return s
    
    def is_perpendicular(self, other):
        c = norm(np.dot(self.direction, other.direction))
        return (c < TOL)
    
    def is_parallel(self, other):
        c = norm(np.cross(self.direction, other.direction))
        return (c < TOL)
    
def _distance_vector(axes1, axes2):
    if axes1.is_parallel(axes2):
        v = axes2.position - axes1.position
        return v - np.dot(v, axes1.direction) * axes1.direction
    else:
        v = axes2.position - axes1.position
        d = np.cross(axes1.direction, axes2.direction)
        return np.dot(v, d) / norm(d) * d

def distance(axes1, axes2, return_vector=False, along=None):
    v = _distance_vector(axes1, axes2)
    if (along is not None):
        return np.abs(np.dot(v, along.direction))
    if (not return_vector):
        return norm(v)
    return v


