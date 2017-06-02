import numpy as np
from rover_state import RoverState
import logging

logger = logging.getLogger('main_app.decisions')
logger.setLevel(logging.DEBUG)


# TODO:  - add documentation to every function
# TODO:  - write the project readme.md report


def get_moves(row, col, max_rows, max_cols, all_moves=False):
    # type: (int, int, int, int) -> list

    if all_moves:
        full_moves = [(row + 1, col), (row - 1, col), (row, col + 1), (row, col - 1),
                      (row - 1, col - 1), (row + 1, col - 1), (row - 1, col + 1), (row + 1, col + 1)]
    else:  # without diagonal moves
        full_moves = [(row + 1, col), (row - 1, col), (row, col + 1), (row, col - 1)]

    return [move for move in full_moves if 0 <= move[0] < max_rows and 0 <= move[1] < max_cols]


def decision_step(Rover):
    # type: (RoverState) -> RoverState

    Rover.update_state()

    logger.debug("Current Mode: {0}".format(Rover.mode))
    logger.debug("Current Commands: {0!s}".format(Rover.commands))

    Rover.print_nav_info()
    Rover.next_cycle()

    return Rover
