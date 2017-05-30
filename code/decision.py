import numpy as np
import queue
from rover_state import RoverState
from perception import distance


def point_is_unexplored(Rover, x, y):
    """
    Returns true if the value of the worldmap at position x,y is (0,0,0)
    :param RoverState Rover: 
    :param int x: 
    :param int y: 
    :return bool: 
    """
    result = (Rover.worldmap[y, x, :] == (0, 0, 0))  # type: np.ndarray
    return result.all()


def point_is_terrain(Rover, x, y):
    """
    Returns true if the value of the worldmap is most probably a terrain point
    :param RoverState Rover: 
    :param int x: 
    :param int y: 
    :return bool: 
    """
    return Rover.worldmap[y, x, 2] > Rover.worldmap[y, x, 0]


def point_is_accessible(Rover, x, y):
    # type: (RoverState, int, int) -> bool
    """ Return true if there is terrain cell in the vicinity of the cell of interest """
    world_size = Rover.worldmap.shape[0]

    x_min, x_max = max(x - 1, 0), min(x + 1, world_size - 1)
    y_min, y_max = max(y - 1, 0), min(y + 1, world_size - 1)
    # if the region centered around point (x,y) has any cell accessible
    # accessible is if the second channel value is larger than the obstacle channel value
    result = (Rover.worldmap[y_min:y_max + 1, x_min:x_max + 1, 2]
              >
              Rover.worldmap[y_min:y_max + 1, x_min:x_max + 1, 0])  # type: np.ndarray

    return result.any()


def find_nearest_unexplored_point(Rover, radius):
    # type: (RoverState, int) -> list

    world_size = Rover.worldmap.shape[0]
    x, y, yaw = int(Rover.pos[0]), int(Rover.pos[1]), Rover.yaw

    x_min, x_max = max(x - radius, 0), min(x + radius, world_size - 1)
    y_min, y_max = max(y - radius, 0), min(y + radius, world_size - 1)

    # search rows for unexplored point
    row_points = [(x_i, y_i) for y_i in [y_min, y_max]
                  for x_i in np.arange(x_min, x_max + 1, 1)
                  if point_is_unexplored(Rover, x_i, y_i) and point_is_accessible(Rover, x_i, y_i)]
    # and point_is_accessible(Rover, x_i, y_i)

    col_points = [(x_i, y_i) for x_i in [x_min, x_max]
                  for y_i in np.arange(y_min, y_max + 1, 1)
                  if point_is_unexplored(Rover, x_i, y_i) and point_is_accessible(Rover, x_i, y_i)]

    # max_radius = max(Rover.worldmap.shape[1] - x, x, Rover.worldmap.shape[0] - y, y) # distance from image edges
    return row_points + col_points


def find_next_point(Rover):
    # type: (RoverState) -> tuple or None

    # world_size = Rover.worldmap.shape[0]
    x, y, yaw = int(Rover.pos[0]), int(Rover.pos[1]), Rover.yaw
    max_radius = max(Rover.worldmap.shape[1] - x, x, Rover.worldmap.shape[0] - y, y)  # distance from image edges
    for r in np.arange(1, max_radius):
        target_destinations = find_nearest_unexplored_point(Rover, r)
        if len(target_destinations) != 0:
            return target_destinations[0]  # return the first point eligible.
            # TODO: select the point that is closes to the yaw of the robot
    # if not points eligible found it probably means we have explored the map.
    return None


def generate_navigation_map(Rover, point_robot, point_destination):
    # type: (RoverState, tuple, tuple) -> np.ndarray
    points_queue = queue.Queue()  # putting there points that should be processed
    # image coordinates have x, y flipped
    destination_position_in_array = (point_destination[1], point_destination[0])  # this is in array form now
    robot_position_in_array = (point_robot[1], point_robot[0])  # this is in array form now

    # setting up navigation map
    navigation_map = np.full(Rover.worldmap[:, :, 0].shape, -1)
    accessibility_condition = (Rover.worldmap[:, :, 2] > Rover.worldmap[:, :, 0])
    navigation_map[accessibility_condition] = 0
    navigation_map[destination_position_in_array] = 1
    points_queue.put(destination_position_in_array)

    while not points_queue.empty():
        row, col = points_queue.get()
        position = (row, col)
        moves = [(row + 1, col), (row - 1, col), (row, col + 1), (row, col - 1)]
        for move in moves:
            if navigation_map[move] == 0:
                navigation_map[move] = navigation_map[position] + 1
                points_queue.put(move)
                if move == robot_position_in_array:
                    return navigation_map
    # end while

    return navigation_map


# region actions

def finish_pending_command(Rover):
    # type: (RoverState) -> None
    if Rover.mode == 'turning-left':
        turn_left(Rover)
    elif Rover.mode == 'turning-right':
        turn_right(Rover)
    elif Rover.mode == 'turning-up':
        turn_up(Rover)
    elif Rover.mode == 'turning-down':
        turn_down(Rover)
    elif Rover.mode.startswith('go-forward'):
        go_forward(Rover)
    elif Rover.mode.startswith('go-yaw'):
        go_yaw(Rover)


def default_action(Rover):
    # type: (RoverState) -> None
    Rover.throttle = Rover.throttle_set
    Rover.steer = 0
    Rover.brake = 0


def turn_left(Rover):
    # type: (RoverState) -> None
    Rover.mode = 'turning-left'
    steering = 180 - Rover.yaw
    steering = steering - 180 if steering > 180 else steering

    if abs(steering) < 1:  # completed turning
        Rover.steer = 0
        Rover.mode = 'finished-command'
    else:
        Rover.steer = min(max(steering, -15), 15)


def turn_right(Rover):
    # type: (RoverState) -> None
    Rover.mode = 'turning-right'
    steering = 0 - Rover.yaw
    steering = steering - 180 if steering > 180 else steering

    if abs(steering) < 1:  # completed turning
        Rover.steer = 0
        Rover.mode = 'finished-command'
    else:
        Rover.steer = min(max(steering, -15), 15)


def turn_up(Rover):
    # type: (RoverState) -> None
    Rover.mode = 'turning-up'
    steering = 90 - Rover.yaw
    steering = steering - 180 if steering > 180 else steering

    if abs(steering) < 1:  # completed turning
        Rover.steer = 0
        Rover.mode = 'finished-command'
    else:
        Rover.steer = min(max(steering, -15), 15)


def turn_down(Rover):
    # type: (RoverState) -> None
    Rover.mode = 'turning-down'
    steering = 270 - Rover.yaw
    steering = steering - 180 if steering > 180 else steering

    if abs(steering) < 1:  # completed turning
        Rover.steer = 0
        Rover.mode = 'finished-command'
    else:
        Rover.steer = min(max(steering, -15), 15)


def go_forward(Rover):
    # type: (RoverState) -> None
    """Will drive the Robot one square ahead"""
    meters = float(Rover.mode.split()[1])  # command is go-forward 10

    # current position of robot
    x, y = Rover.pos[0], Rover.pos[1]

    if Rover.departure_point is None:
        Rover.departure_point = (x, y)

    # check the distance the robot has traveled
    if meters - distance((x, y), Rover.departure_point) < 1.0 and Rover.vel > 2.0:
        hit_brakes(Rover)
    if meters - distance((x, y), Rover.departure_point) < 0.5:
        hit_brakes(Rover)
        Rover.departure_point = None  # reached destination
        Rover.mode = 'finished-command'
    if meters - distance((x, y), Rover.departure_point) >= 1.0:
        Rover.throttle = 0.2
        Rover.mode = Rover.commands[0]


def go_yaw(Rover):
    # type: (RoverState) -> None
    """Will drive the Robot one square ahead"""
    target_yaw = float(Rover.mode.split()[1])  # command is go-forward 10

    steering = target_yaw - Rover.yaw
    steering = steering - 180 if steering > 180 else steering

    if abs(steering) < 0.1:  # completed turning
        Rover.steer = 0
        Rover.mode = 'finished-command'
    else:
        Rover.steer = min(max(steering, -15), 15)
        Rover.mode = Rover.commands[0]


def full_throttle(Rover):
    # type: (RoverState) -> None
    if Rover.vel < Rover.max_vel:
        # Set throttle value to throttle setting
        Rover.throttle = Rover.throttle_set
    else:  # Else coast
        Rover.throttle = 0
    Rover.brake = 0
    Rover.mode = 'forward'


def hit_brakes(Rover):
    # type: (RoverState) -> None
    # Set mode to "stop" and hit the brakes!
    Rover.throttle = 0
    # Set brake to stored brake value
    Rover.brake = Rover.brake_set
    Rover.steer = 0
    # Rover.mode = 'stop'


def steer(Rover):
    # type: (RoverState) -> None
    Rover.steer = np.clip(np.mean(Rover.nav_angles) * 180 / np.pi, -15, 15)


def escape_manoeuvre(Rover):
    # type: (RoverState) -> None
    Rover.commands = ['go-yaw ' + str(Rover.yaw + 15.0), 'go-forward 1']


def navigate(Rover):
    # type: (RoverState) -> None
    pass


# endregion

# region conditions

def can_see_terrain(Rover):
    # type: (RoverState) -> bool
    return Rover.nav_angles is not None


def clear_road_ahead(Rover):
    # type: (RoverState) -> bool
    return len(Rover.nav_angles) >= Rover.stop_forward


def sufficient_road_ahead(Rover):
    # type: (RoverState) -> bool
    return len(Rover.nav_angles) >= Rover.go_forward


def is_moving_forward(Rover):
    # type: (RoverState) -> bool
    return Rover.mode == 'forward'


def is_stopping(Rover):
    # type: (RoverState) -> bool
    return Rover.mode == 'stop'


def still_moving(Rover):
    # type: (RoverState) -> bool
    return Rover.vel > 0.2


def is_executing_command(Rover):
    # type: (RoverState)->bool
    if Rover.mode == 'turning-left' or Rover.mode == 'turning-right':
        return True

    if Rover.mode == 'turning-up' or Rover.mode == 'turning-down':
        return True

    if Rover.mode.startswith('go-forward') or Rover.mode.startswith('go-yaw'):
        return True

    return False


# endregion


# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):
    # type: (RoverState) -> RoverState
    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    print("Current Mode: ", Rover.mode)
    if Rover.mode == 'waiting-command':
        if len(Rover.commands) != 0:  # there are still commands to execute
            Rover.mode = Rover.commands[0]  # this is one of the known commands that is_executing knows to handle
        else:  # no commands so lets find new points
            if not sufficient_road_ahead(Rover):  # try to move the robot to an open space.
                escape_manoeuvre(Rover)
            elif Rover.navigation_map is None:  # sufficient look but no map to navigate create one
                target_point = find_next_point(Rover)
                Rover.navigation_map = generate_navigation_map(Rover,
                                                               (int(Rover.pos[0]), int(Rover.pos[1])),
                                                               target_point)
                print(Rover.navigation_map)
            else:  # there is a map, so use it to navigate
                pass
    elif Rover.mode == 'finished-command':
        Rover.commands.pop(0)
        Rover.mode = 'waiting-command'
    elif is_executing_command(Rover):
        finish_pending_command(Rover)
    # if can_see_terrain(Rover):
    #     # Check for Rover.mode status
    #     if is_moving_forward(Rover):
    #         # Check the extent of navigable terrain
    #         if clear_road_ahead(Rover):
    #             # If mode is forward, navigable terrain looks good then full throttle
    #             full_throttle(Rover)
    #             # Set steering to average angle clipped to the range +/- 15
    #             steer(Rover)
    #         # If there's a lack of navigable terrain pixels then go to 'stop' mode
    #         else:
    #             hit_brakes(Rover)
    #
    #     # If we're already in "stop" mode then make different decisions
    #     elif is_stopping(Rover):
    #         # If we're in stop mode but still moving keep braking
    #         if still_moving(Rover):
    #             hit_brakes(Rover)
    #         # If we're not moving (vel < 0.2) then do something else
    #         else:
    #             # Now we're stopped and we have vision data to see if there's a path forward
    #             if not sufficient_road_ahead(Rover):
    #                 Rover.throttle = 0
    #                 # Release the brake to allow turning
    #                 Rover.brake = 0
    #                 # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
    #                 Rover.steer = -15  # Could be more clever here about which way to turn
    #             # If we're stopped but see sufficient navigable terrain in front then go!
    #             else:
    #                 full_throttle(Rover)
    #                 steer(Rover)
    # # Just to make the rover do something
    # # even if no modifications have been made to the code
    # else:
    #     default_action(Rover)

    return Rover
