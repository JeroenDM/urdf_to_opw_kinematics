#!/usr/bin/env python
import numpy as np
from numpy.linalg import norm

TOL = 1e-6

def angle(v1, v2):
    cos = np.dot(v1, v2) / (norm(v1) * norm(v2))
    sin = norm(np.cross(v1, v2))
    return np.arctan2(sin, cos)

class Axis:
    def __init__(self, point, direction):
        self.p = point
        self.d = direction / norm(direction)
    
    def __str__(self):
        s = "[Axis]\tpoint: "
        s += str(self.p) + "\tdir: "
        s += str(self.d) + "\n"
        return s
    
    def is_perpendicular(self, other):
        c = norm(np.dot(self.d, other.d))
        return (c < TOL)
    
    def is_parallel(self, other):
        c = norm(np.cross(self.d, other.d))
        return (c < TOL)
    
    def shortest_distance_vector(self, other):
        if self.is_parallel(other):
            v = other.p - self.p
            return v - np.dot(v, self.d) * self.d
        else:
            v = other.p - self.p
            d = np.cross(self.d, other.d)
            return np.dot(v, d) / norm(d) * d
    
    def shortest_distance(self, other):
        return norm(self.shortest_distance_vector(other))