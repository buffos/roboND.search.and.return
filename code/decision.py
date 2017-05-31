import numpy as np
from utilities import create_navigation_map_string
from rover_state import RoverState
import logging

logger = logging.getLogger('main_app.decisions')
logger.setLevel(logging.DEBUG)


def get_moves(row, col, max_rows, max_cols, all_moves=False):
    # type: (int, int, int, int) -> list

    if all_moves:
        full_moves = [(row + 1, col), (row - 1, col), (row, col + 1), (row, col - 1),
                      (row - 1, col - 1), (row + 1, col - 1), (row - 1, col + 1), (row + 1, col + 1)]
    else:  # without diagonal moves
        full_moves = [(row + 1, col), (row - 1, col), (row, col + 1), (row, col - 1)]

    return [move for move in full_moves if 0 <= move[0] < max_rows and 0 <= move[1] < max_cols]


# region map creation

# endregion

# region conditions

def is_clear_ahead(Rover):
    # type: (RoverState) -> bool
    front_pixels = np.sum(Rover.terrain[120:160, 150:170])
    clear = front_pixels > 500  # type: bool
    print("FRONT PIXELS: ", front_pixels)
    if not clear:
        print("FRONT PIXELS ARE FEW: ", front_pixels)
    return clear


# endregion

# region actions








# endregion

# region find next point

def find_next_point(Rover):
    # type: (RoverState) -> np.ndarray
    pass


# endregion


def decision_step(Rover):
    # type: (RoverState) -> RoverState

    Rover.update_state()

    if Rover.base is None:
        assert (Rover.pos is not None)
        Rover.base = Rover.pos

    logger.debug("Current Mode: {0}".format(Rover.mode))
    logger.debug("Current Commands: {0!s}".format(Rover.commands))
    print("Current Mode: {0}".format(Rover.mode))

    left, center, right = Rover.get_navigation_angles()
    print("CENTER: ", center)
    print("RIGHT: ", right)
    print("LEFT: ", left)
    print("Velocity", Rover.vel)

    trapped = 0
    trapped = trapped + 1 if center < 30 else trapped
    trapped = trapped + 1 if left < 30 else trapped
    trapped = trapped + 1 if right < 30 else trapped

    print(trapped)
    print ("Stuck counter: ", Rover.stuck_counter)

    Rover.generate_exploration_map()
    # logger.debug(create_navigation_map_string(Rover.navigation_map, 50, 50, Rover.pos))

    if Rover.vel > 2.0:
        Rover.brake = Rover.vel  # do not speed and brake as hard as speed

    if 1500 > Rover.stuck_counter > 1000:
        print("STUCK - LEFT BACK")
        Rover.steer = 15
        Rover.throttle = -5
    elif 2000 >Rover.stuck_counter > 1500:  # try steering the other way
        print("STUCK - RIGHT BACK")
        Rover.steer = -15
        Rover.throttle = -5
    elif 2500> Rover.stuck_counter > 2000:  # try steering the other way
        print("FRONT LEFT-STUCK")
        Rover.steer = 15
        Rover.throttle = 0
    elif 3000> Rover.stuck_counter > 2500:  # try steering the other way
        print("FRONT RIGHT-STUCK")
        Rover.steer = -15
        Rover.throttle = 0
    elif Rover.stuck_counter > 3000:
        Rover.stuck_counter = 1001 # loop again all strategies

    elif trapped > 1 and center < 100:  # so no forward
        if Rover.vel > 0:
            Rover.brake = Rover.brake_set  # stop
        else:
            Rover.steer = -15  # steer right. brakes are already set to 0 by update state

    elif trapped > 1 and center > 100:
        if Rover.vel > 1:
            pass
        else:
            Rover.throttle = 0.1  # go slowly its narrow
    elif center > 300 and left < 300:
        if Rover.vel < 1:
            Rover.throttle = 0.5
        else:
            pass
    elif center > 300 and left > 300:
        Rover.steer = 15  # try to follow left wall
        if Rover.vel < 1:
            Rover.throttle = 0.5
        else:
            pass
    elif left > 300:
        Rover.steer = 15
        if Rover.vel < 1:
            Rover.throttle = 0.5
        else:
            pass
    elif right > 300 and left < 150:
        Rover.steer = -5
        if Rover.vel < 1:
            Rover.throttle = 0.5
        else:
            pass
    else:
        Rover.steer = -10

    # Rover.next_cycle()

    return Rover
