from math import sqrt


def distance(p_1, p_2):
    return sqrt((p_1[0] - p_2[0]) ** 2 + (p_1[1] - p_2[1]) ** 2)