import numpy as np
from utilities import distance


# Define RoverState() class to retain rover state parameters
class RoverState:
    def __init__(self):
        # To record the start time of navigation
        self.start_time = None  # type: float
        # To record total duration of navigation
        self.total_time = None  # type: float
        # Current camera image
        self.img = None  # type: np.ndarray
        # Current position (x, y)
        self.pos = None, None  # type: tuple
        # Current yaw angle
        self.yaw = None
        # Current pitch angle
        self.pitch = None  # type: float
        # Current roll angle
        self.roll = None  # type: float
        # Current velocity
        self.vel = None  # type: float
        # Current steering angle
        self.steer = 0  # type: float
        # Current throttle value
        self.throttle = 0  # type: float
        # Current brake value
        self.brake = 0  # type: float
        # Angles of navigable terrain pixels
        self.nav_angles = None  # type: np.ndarray
        # Distances of navigable terrain pixels
        self.nav_dists = None  # type: np.ndarray
        # Ground truth worldmap
        self.ground_truth = None  # type: np.ndarray
        # Current mode (can be forward or stop)
        self.mode = 'waiting-command'  # type: str
        # Throttle setting when accelerating
        self.throttle_set = 0.2  # type: float
        # Brake setting when braking
        self.brake_set = 2  # type: int
        # The threshold_stop_forward and threshold_go_forward fields below represent total count
        # of navigable terrain pixels.  This is a very crude form of knowing
        # when you can keep going and when you should stop.  Feel free to
        # get creative in adding new fields or modifying these!

        # Threshold to initiate stopping
        self.threshold_stop_forward = 50  # type: int
        # Threshold to go forward again
        self.threshold_go_forward = 500  # type: int
        # Maximum velocity (meters/second)
        self.max_vel = 2  # type: int
        # Image output from perception step
        # Update this image to display your intermediate analysis steps
        # on screen in autonomous mode
        self.vision_image = np.zeros((160, 320, 3), dtype=np.float)  # type: np.ndarray
        # To store the actual sample positions
        self.samples_pos = None
        # To store the initial count of samples
        self.samples_to_find = 0  # To store the initial count of samples
        # To count the number of samples found
        self.samples_found = 0  # type: int
        # Will be set to telemetry value data["near_sample"]
        self.near_sample = 0  # type: int
        # Will be set to telemetry value data["picking_up"]
        self.picking_up = 0  # type: int
        # Set to True to trigger rock pickup
        self.send_pickup = False  # type: bool
        # a list of pending commands for the robot
        self.commands = ['go-yaw 0.0', 'go-yaw 90.0', 'go-yaw 180.0', 'go-yaw 270.0', 'go-yaw 0.0']  # type: list
        # the departure_point the Robot started from
        self.departure_point = None  # type: tuple
        # robot starting point
        self.base = None  # type: tuple
        # the thresholded terrain
        self.terrain = None  # type: np.ndarray
        # a flag the Robot is on unstuck mode
        self.escaping = False  # type: bool
        # a counter for identifying stuck vehicle
        self.stuck_counter = 0  # type: int
        self.previous_position = None  # type: np.ndarray
        # Worldmap
        # Update this image with the positions of navigable terrain
        # obstacles and rock samples
        self.worldmap = np.zeros((200, 200, 3), dtype=np.float)  # type: np.ndarray
        # the current navigation map
        self.navigation_map = None  # type: np.ndarray
        # marks every point the robot has
        self.visited_map = np.zeros((200, 200), dtype=np.float)  # type: np.ndarray

    def update_state(self):
        x, y = self.pos
        self.visited_map[int(y), int(x)] = 1  # mark the position as visited
        # just to make sure we are not stuck in any loop with brakes on
        # action will only set those and do not need to unset them.
        print("DISTANCE FROM PREVIOUS POINT: ", distance(self.pos, self.previous_position))
        if self.stuck_counter == 0 :
            # change previous position when the robot has traveled
            self.previous_position = self.pos
        self.stuck_counter = self.stuck_counter + 1 if distance(self.pos, self.previous_position) < 0.5 else 0

        self.brake = 0
        self.throttle = 0
        self.steer = 0

    def next_cycle(self):
        if self.mode == 'waiting-command':
            if len(self.commands) != 0:  # there are still commands to execute
                self.mode = self.commands[0]  # this is one of the known commands that is_executing knows to handle
            else:  # no commands so lets find new points
                pass
        elif self.mode == 'finished-command':
            self.commands.pop(0)
            self.mode = 'waiting-command'
        elif self.is_executing_command():
            self.finish_pending_command()

    def finish_pending_command(self):
        """
        Interpreting test commands to actual functions
        :return: 
        """
        if self.mode.startswith('go-forward'):
            self.go_forward()
        elif self.mode.startswith('go-yaw'):
            self.go_yaw()

    def is_executing_command(self):
        """
        Is a known command executed
        :return: 
        """
        if self.mode.startswith('go-forward') or self.mode.startswith('go-yaw'):
            return True
        return False

    def go_yaw(self):
        """Will drive the Robot one square ahead"""
        if self.vel > 0:
            self.throttle = 0
            self.brake = self.brake_set
            return
        else:
            self.brake = 0

        target_yaw = float(self.mode.split()[1])  # command is go-yaw angle. always in [-pi pi]
        current_yaw = self.yaw - 360 if self.yaw > 180 else self.yaw

        # choose the shortest path for rotating
        if abs(target_yaw - current_yaw) < abs(360 - target_yaw + current_yaw):
            steering = target_yaw - current_yaw
        else:
            steering = -(360 - target_yaw + current_yaw)

        if abs(steering) < 0.1:  # completed turning
            self.steer = 0
            self.mode = 'finished-command'
        else:
            self.throttle = 0
            self.steer = min(max(steering, -15), 15)
            self.mode = self.commands[0]

    def go_forward(self):
        """Will drive the Robot one square ahead"""

        self.brake = 0
        meters = float(self.mode.split()[1])  # command is go-forward 10

        # current position of robot
        x, y = self.pos[0], self.pos[1]

        if self.departure_point is None:
            self.departure_point = (x, y)

        current_distance = distance((x, y), self.departure_point)
        print(current_distance)

        # check the distance the robot has traveled
        if meters - current_distance < 0.1:
            self.departure_point = None  # reached destination
            self.throttle = 0
            self.mode = 'finished-command'
        elif meters - current_distance >= 1.0:
            self.throttle = 1
            self.mode = self.commands[0]
        else:
            self.throttle = 0.5
            self.mode = self.commands[0]

    def collect_rock(self):
        if self.near_sample and self.vel == 0 and not self.picking_up:
            self.send_pickup = True
        elif self.near_sample and self.vel > 0:
            self.mode = 'waiting-command'  # this will suspend commands

    def get_navigation_angles(self):
        """
        Return how many driving angles are available for center left and right
        :return: 
        """
        center = len(self.nav_angles[(-0.1 <= self.nav_angles) & (self.nav_angles <= 0.1)])
        left = len(self.nav_angles[self.nav_angles > 0.1])
        right = len(self.nav_angles[self.nav_angles < -0.1])

        return left, center, right

    def generate_exploration_map(self):
        # init the map
        self.navigation_map = np.full(self.worldmap[:, :, 0].shape, -1)  # unknown or empty
        accessibility_condition = (self.worldmap[:, :, 2] > self.worldmap[:, :, 0]) & (self.worldmap[:, :, 2] > 10)
        obstacles_condition = (self.worldmap[:, :, 0] > self.worldmap[:, :, 2]) & (self.worldmap[:, :, 0] > 10)
        visited_condition = (self.visited_map[:, :] == 1)
        self.navigation_map[accessibility_condition] = 0  # terrain
        self.navigation_map[obstacles_condition] = -2  # obstacles
        self.navigation_map[visited_condition] = 1  # visited
