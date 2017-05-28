import numpy as np


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
        self.mode = 'forward'  # type: str
        # Throttle setting when accelerating
        self.throttle_set = 0.2  # type: float
        # Brake setting when braking
        self.brake_set = 10  # type: int
        # The stop_forward and go_forward fields below represent total count
        # of navigable terrain pixels.  This is a very crude form of knowing
        # when you can keep going and when you should stop.  Feel free to
        # get creative in adding new fields or modifying these!

        # Threshold to initiate stopping
        self.stop_forward = 50  # type: int
        # Threshold to go forward again
        self.go_forward = 500  # type: int
        # Maximum velocity (meters/second)
        self.max_vel = 2  # type: int
        # Image output from perception step
        # Update this image to display your intermediate analysis steps
        # on screen in autonomous mode
        self.vision_image = np.zeros((160, 320, 3), dtype=np.float)  # type: np.ndarray
        # Worldmap
        # Update this image with the positions of navigable terrain
        # obstacles and rock samples
        self.worldmap = np.zeros((200, 200, 3), dtype=np.float)  # type: np.ndarray
        # To store the actual sample positions
        self.samples_pos = None
        # To count the number of samples found
        self.samples_found = 0  # type: int
        # Will be set to telemetry value data["near_sample"]
        self.near_sample = 0  # type: int
        # Will be set to telemetry value data["picking_up"]
        self.picking_up = 0  # type: int
        # Set to True to trigger rock pickup
        self.send_pickup = False  # type: bool
