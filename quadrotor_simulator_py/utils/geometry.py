import math
import numpy as np

def unroll(x):
    x = math.fmod(x, 2 * math.pi)
    if x < 0:
        x += 2 * math.pi
    return x

def normalize(x):
    x = math.fmod(x + math.pi, 2 * math.pi)
    if x < 0:
        x += 2 * math.pi
    return x - math.pi

def shortest_angular_distance(a0, a1):
    result = unroll(unroll(a0) - unroll(a1))
    if result > math.pi:
        result -= 2*math.pi - result
    return normalize(result)
