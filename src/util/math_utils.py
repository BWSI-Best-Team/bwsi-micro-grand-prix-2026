""" Math helpers """
import math


# wrap angle to (-pi, pi]
def wrap_pi(a):
    while a > math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a
