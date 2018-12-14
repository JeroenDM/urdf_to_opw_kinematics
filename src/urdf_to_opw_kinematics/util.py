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
    
    def shortest_distance_vector(self, other):
        if self.is_parallel(other):
            v = other.position - self.position
            return v - np.dot(v, self.direction) * self.direction
        else:
            v = other.position - self.position
            d = np.cross(self.direction, other.direction)
            return np.dot(v, d) / norm(d) * d
    
    def shortest_distance(self, other):
        return norm(self.shortest_distance_vector(other))