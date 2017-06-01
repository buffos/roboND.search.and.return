import numpy as np
from utilities import create_navigation_map_string
from rover_state import RoverState
import logging

logger = logging.getLogger('main_app.decisions')
logger.setLevel(logging.DEBUG)


# TODO:  - create Mapping and Stuck Modes
# TODO:  - better explorer the map.
# TODO:  - try to turn sharper when left is available


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

def find_next_point():
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

    Rover.generate_exploration_map()
    # logger.debug(create_navigation_map_string(Rover.navigation_map, 50, 50, Rover.pos))

    Rover.print_nav_info()
    Rover.next_cycle()

    return Rover
