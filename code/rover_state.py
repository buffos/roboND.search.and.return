import numpy as np
from utilities import distance, yaw_from_to
from math import atan2, degrees


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
        # Angles (polar) of terrain, obstacles and rocks
        # Distances of navigable terrain pixels
        self.nav_angles = None  # type: np.ndarray
        self.nav_dists = None  # type: np.ndarray
        self.obs_angles = None  # type: np.ndarray
        self.obs_dists = None  # type: np.ndarray
        self.rock_angles = None  # type: np.ndarray
        self.rock_dists = None  # type: np.ndarray
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
        # This is to monitor if picking_up was triggered
        self.started_picking_up = False  # type: bool
        # Set to True to trigger rock pickup
        self.send_pickup = False  # type: bool
        # a list of pending commands for the robot
        self.commands = ['mapping']  # type: list
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
        self.stuck_threshold = 500  # type: int
        self.previous_position = None  # type: np.ndarray
        # it has a position value IF I am seeing a rock on camera.
        self.seen_rock = None  # type: np.ndarray
        self.is_collecting = False  # type: bool
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
        if self.stuck_counter == 0:
            # change previous position when the robot has traveled
            self.previous_position = self.pos

        self.stuck_counter = self.stuck_counter + 1 if distance(self.pos, self.previous_position) < 0.5 else 0

        # do not speed and brake as hard as speed
        self.brake = self.vel if self.vel > 2.0 else 0
        self.throttle = 0
        self.steer = 0

    def next_cycle(self):
        # handling stuck robot
        if self.stuck_counter > self.stuck_threshold and self.mode != 'unstuck' and not self.picking_up:
            self.commands = ['unstuck'] + self.commands  # put it in front
            self.mode = 'waiting-command'
        elif self.stuck_counter < self.stuck_threshold and self.mode == 'unstuck':
            self.mode = 'finished-command'

        if self.seen_rock is not None and not self.is_collecting:
            self.commands = ['collecting'] + self.commands
            self.mode = 'waiting-command'
            self.is_collecting = True

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
        elif self.mode == 'mapping':
            self.mapping()
        elif self.mode == 'unstuck':
            self.unstuck()
        elif self.mode == 'collecting':
            self.collect()

    def is_executing_command(self):
        """
        Is a known command executed
        :return: 
        """
        if self.mode.startswith('go-forward') or self.mode.startswith('go-yaw'):
            return True
        if self.mode in ['mapping', 'unstuck', 'stopping', 'collecting']:
            return True
        return False

    def mapping(self):
        trapped = self.trapped()
        left, center, right = self.get_navigation_angles()
        bottom_close_l, middle_far_l, up_far_l = self.get_obstacles_left()
        bottom_close_r, middle_far_r, up_far_r = self.get_obstacles_right()
        center_close, center_far, center_left, center_right = self.get_obstacles_center()

        if trapped > 1 and center < 100:  # so no forward
            if self.vel > 0:
                self.brake = self.brake_set  # stop
            else:
                self.steer = -15  # steer right. brakes are already set to 0 by update state

        elif trapped > 1 and center > 100:
            if self.vel > 1:
                pass
            else:
                self.throttle = 0.1  # go slowly its narrow
        elif center_close > 50:  # GETTING CLOSE TO CRASH
            if self.vel != 0:
                self.brake = self.brake_set
            else:  # we stopped
                self.steer = np.random.uniform(-12, -15)

        elif bottom_close_l == 0 and middle_far_l == 0 and up_far_l == 0:
            # self.steer = np.random.uniform(8, 12) * np.random.choice([-1, 1], 1)[0]
            # to avoid running into circles on open spaces form [-12,-8] U [8, 12]
            self.steer = np.random.uniform(12, 15)
            self.drive_safely()
        elif bottom_close_l > 0:  # to close to the left wall
            self.steer = np.random.uniform(-12, -15)
            self.drive_safely()
        elif bottom_close_r > 0:  # to close to the right wall
            self.steer = np.random.uniform(12, 15)
            self.drive_safely()
        elif middle_far_l < 80:  # intersection -> gap in the middle
            self.steer = np.random.uniform(12, 15)
            self.drive_safely()
        elif center > 300 and left < 301:
            self.drive_safely()
        elif center > 300 and left > 300:
            self.steer = np.random.uniform(12, 15)  # try to follow left wall
            self.drive_safely()
        elif left > 300:
            self.steer = np.random.uniform(12, 15)
            self.drive_safely()
        elif right > 300 and left < 150:
            self.steer = np.random.uniform(-3, -6)
            self.drive_safely()
        else:
            self.steer = self.steer = np.random.uniform(-10, -15)

    def drive_safely(self):
        if self.vel < 1:
            self.throttle = 0.5
        else:
            pass

    def go_yaw(self):
        """Will drive the Robot one square ahead"""
        if self.vel > 0:
            self.brake = self.brake_set
            return
        else:
            pass

        target_yaw = float(self.mode.split()[1])  # command is go-yaw angle. always in [-pi pi]
        steering = yaw_from_to(self.yaw, target_yaw)

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
        else:
            self.throttle = 0.5
            self.mode = self.commands[0]

    def unstuck(self):
        if self.mode == 'collecting':
            extra_wait = 300
            if self.stuck_counter < self.stuck_threshold + extra_wait:
                return

        min_strategy = 1
        max_strategy = 7
        strategy = np.random.randint(min_strategy, max_strategy + 1)
        self.unstuck_strategy(strategy)

    def unstuck_strategy(self, strategy):
        if strategy == 1:
            # try steering the other way
            print("FRONT STUCK")
            self.steer = 0
            self.throttle = 10
        elif strategy == 2:
            # try steering the other way
            print("STUCK - RIGHT BACK")
            self.steer = np.random.uniform(-13, -15)
            self.throttle = -10
        elif strategy == 3:
            print("STUCK - LEFT BACK")
            self.steer = np.random.uniform(13, 15)
            self.throttle = -10
        elif strategy == 4:
            # try steering the other way
            print("FRONT RIGHT-STUCK")
            self.steer = np.random.uniform(-13, -15)
            self.throttle = 0
        elif strategy == 5:
            # try steering the other way
            print("FRONT LEFT-STUCK")
            self.steer = np.random.uniform(13, 15)
            self.throttle = 0
        elif strategy == 6:
            # try steering the other way
            print("FRONT RIGHT-WITH-SPEED-STUCK")
            self.steer = np.random.uniform(-13, -15)
            self.throttle = 10
        elif strategy == 7:
            # try steering the other way
            print("FRONT LEFT-WITH-SPEED-STUCK")
            self.steer = np.random.uniform(13, 15)
            self.throttle = 10

    def collect(self):
        self.is_collecting = True

        if self.picking_up:
            self.started_picking_up = True
        elif self.near_sample and not self.picking_up:
            self.brake = self.brake_set
            self.throttle = 0
            self.send_pickup = True
        elif not self.picking_up and self.started_picking_up:
            # finished picking up
            self.started_picking_up = False
            self.seen_rock = None
            self.is_collecting = False
            self.mode = 'finished-command'
        elif len(self.rock_angles) > 0:  # I can see the rock
            yaw_r = degrees(atan2(self.seen_rock[1] - self.pos[1], self.seen_rock[0] - self.pos[0]))
            yaw_diff = yaw_from_to(self.yaw, yaw_r)

            # self.steer = np.clip(yaw_diff, -15, 15)
            self.steer = np.clip(np.mean(self.rock_angles * 180 / np.pi), -15, 15)
            print("YAW DIFFERENCE {0}".format(yaw_diff))
            print("ROCK ANGLE {0}".format(self.steer))
            if self.vel > 1.0:
                self.brake = 1
            elif 0.5 <= self.vel <= 1:
                self.throttle = 0.2
            elif self.vel < 0.5:
                self.throttle = 1
        else:
            if len(self.nav_angles) < self.threshold_stop_forward:
                self.throttle = -1
            else:  # lost it... continue
                self.started_picking_up = False
                self.seen_rock = None
                self.is_collecting = False
                self.mode = 'finished-command'

    def get_navigation_angles(self):
        """
        Return how many driving angles are available for center left and right
        :return: 
        """
        center = len(self.nav_angles[(-0.1 <= self.nav_angles) & (self.nav_angles <= 0.1)])
        left = len(self.nav_angles[self.nav_angles > 0.1])
        right = len(self.nav_angles[self.nav_angles < -0.1])

        return left, center, right

    def get_obstacles_left(self):
        bottom_close = np.count_nonzero(self.vision_image[145:160, 150:155, 0])
        middle_far = np.count_nonzero(self.vision_image[135:145, 130:145, 0])
        up_far = np.count_nonzero(self.vision_image[125:135, 130:145, 0])
        return bottom_close, middle_far, up_far

    def get_obstacles_right(self):
        bottom_close = np.count_nonzero(self.vision_image[145:160, 165:170, 0])
        middle_far = np.count_nonzero(self.vision_image[135:145, 175:190, 0])
        up_far = np.count_nonzero(self.vision_image[125:135, 175:190, 0])
        return bottom_close, middle_far, up_far

    def get_obstacles_center(self):
        center_close = np.count_nonzero(self.vision_image[140:150, 150:170, 0])
        center_far = np.count_nonzero(self.vision_image[120:140, 150:170, 0])
        center_left = np.count_nonzero(self.vision_image[140:150, 130:160, 0])
        center_right = np.count_nonzero(self.vision_image[140:150, 160:190, 0])
        return center_close, center_far, center_left, center_right

    def generate_exploration_map(self):
        # init the map
        self.navigation_map = np.full(self.worldmap[:, :, 0].shape, -1)  # unknown or empty
        accessibility_condition = (self.worldmap[:, :, 2] > self.worldmap[:, :, 0]) & (self.worldmap[:, :, 2] > 10)
        obstacles_condition = (self.worldmap[:, :, 0] > self.worldmap[:, :, 2]) & (self.worldmap[:, :, 0] > 10)
        visited_condition = (self.visited_map[:, :] == 1)
        self.navigation_map[accessibility_condition] = 0  # terrain
        self.navigation_map[obstacles_condition] = -2  # obstacles
        self.navigation_map[visited_condition] = 1  # visited

    def trapped(self):
        left, center, right = self.get_navigation_angles()

        trapped = 0
        trapped = trapped + 1 if center < 30 else trapped
        trapped = trapped + 1 if left < 30 else trapped
        trapped = trapped + 1 if right < 30 else trapped
        return trapped

    def print_nav_info(self):
        left, center, right = self.get_navigation_angles()
        print("Current Mode: {0}".format(self.mode))
        print("Commands: {0!s}".format(self.commands))
        print("CENTER: {0}  RIGHT: {1} LEFT: {2}".format(center, right, left))
        print("Stuck counter: {0} , Trapped: {1}".format(self.stuck_counter, self.trapped()))
        print("Velocity: {0} , Throttle: {1} Brakes: {2}".format(self.vel, self.throttle, self.brake))
        print("Obstacles Left  bottom-close {0} , middle-far {1}, up-far {2}".format(*self.get_obstacles_left()))
        print("Obstacles right  bottom-close {0} , middle-far {1}, up-far {2}".format(*self.get_obstacles_right()))
        print("Obstacles center  center-close {0} , center-far {1},"
              " center-left {2}, center-right {3}".format(*self.get_obstacles_center()))

        if self.seen_rock is not None:
            print("-------------ROCK ON SIGHT ----------------------")
