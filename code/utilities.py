from math import sqrt, cos, sin, acos, degrees, radians, copysign
import numpy as np


def distance(p_1, p_2):
    if p_1 is None or p_2 is None:
        return 0
    return sqrt((p_1[0] - p_2[0]) ** 2 + (p_1[1] - p_2[1]) ** 2)


def yaw_from_to(y_1, y_2):
    y_1 = y_1 - 360 if y_1 > 180 else y_1
    y_2 = y_2 - 360 if y_2 > 180 else y_2
    # print (y_1,y_2)
    result = y_2 - y_1
    return result if result < 180 else 180 - result


def look_to_point(target, robot_position, robot_yaw):
    """ This will give how much to rotate to point to another object. Robot yaw is LOCAL"""
    robot_yaw_rad = radians(robot_yaw)
    robot_vector = (cos(robot_yaw_rad), sin(robot_yaw_rad))
    target_vector = (target[0] - robot_position[0], target[1] - robot_position[1])
    radius = distance(target, robot_position)
    dot_product = np.dot(np.asarray(target_vector), np.asarray(robot_vector))
    determinant =  target_vector[1]*robot_vector[0] - target_vector[0]*robot_vector[1]
    result = acos(dot_product / radius) if radius > 0 else 0
    result = copysign(result, determinant)

    return degrees(result) , radius


def create_navigation_map_string(the_map, region_x=0, region_y=0, robot_position=(0, 0), destination=None):
    """ 
    Generate a text string map representation for debugging 
    
    robot_position : in x, y coordinates and not in array
    """

    cols = int(the_map.shape[1] / 2) if region_x == 0 else int(region_x / 2)
    rows = int(the_map.shape[0] / 2) if region_y == 0 else int(region_y / 2)

    robot_col, robot_row = int(robot_position[0]), int(robot_position[1])

    col_min, col_max = max(0, robot_col - cols), min(the_map.shape[1], robot_col + cols)
    row_min, row_max = max(0, robot_row - rows), min(the_map.shape[0], robot_row + rows)

    str_map = ''
    for r in range(row_min, row_max):
        row_sting = ''
        for c in range(col_min, col_max):
            if (c, r) == robot_position:
                row_sting = row_sting + ' | R' + str(map_code_to_text(the_map[r, c]))
            elif destination is not None and (c, r) == destination:
                row_sting = row_sting + ' | D'
            else:
                row_sting = row_sting + ' | ' + str(map_code_to_text(the_map[r, c]))
        str_map += row_sting + '\n'

    return str_map


def map_code_to_text(code):
    code_dict = {
        1: 'V',
        0: 'O',
        -2: 'X',
        -1: '--'
    }
    return code_dict[code]

