import numpy as np
import cv2
from rover_state import RoverState
from utilities import distance


# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_threshold(img, rgb_thresh=(140, 140, 140)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:, :, 0])
    image_y, image_x, _ = img.shape
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:, :, 0] > rgb_thresh[0]) & (img[:, :, 1] > rgb_thresh[1]) & (img[:, :, 2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[80:, 80:240][above_thresh[80:, 80:240]] = 1
    # Return the binary image
    return color_select


def obstacles_threshold(img, threshold_high=(140, 140, 140), threshold_low=(30, 30, 30)):
    obstacles_mask = np.zeros_like(img[:, :, 0])
    image_y, image_x, _ = img.shape
    below_thresh = (img[:, :, 0] < threshold_high[0]) & \
                   (img[:, :, 1] < threshold_high[1]) & \
                   (img[:, :, 2] < threshold_high[2]) & \
                   (threshold_low[0] < img[:, :, 0]) & \
                   (threshold_low[1] < img[:, :, 1]) & \
                   (threshold_low[2] < img[:, :, 2])
    # temp_select[below_thresh] = 1
    obstacles_mask[80:, 80:240][below_thresh[80:, 80:240]] = 1

    return obstacles_mask


def rocks_threshold_2(img):
    low_yellow = np.array([20, 100, 100], dtype="uint8")
    high_yellow = np.array([255, 255, 255], dtype="uint8")

    # convert to HSV space
    img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV, 3)
    # mask yellow values
    mask_rock = cv2.inRange(img_hsv, low_yellow, high_yellow)
    return mask_rock


def rocks_threshold(img, threshold_low=(100, 100, 0), threshold_high=(160, 160, 40)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:, :, 0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = ((img[:, :, 0] > threshold_low[0]) & (img[:, :, 0] < threshold_high[0]) &
                    (img[:, :, 1] > threshold_low[1]) & (img[:, :, 1] < threshold_high[1]) &
                    (img[:, :, 2] > threshold_low[2]) & (img[:, :, 2] < threshold_high[2]))
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    return color_select


# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel ** 2 + y_pixel ** 2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles


# Define a function to apply a rotation to pixel positions
def rotate_pix(x_pixel, y_pixel, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    # Apply a rotation
    x_pixel_rotated = (x_pixel * np.cos(yaw_rad)) - (y_pixel * np.sin(yaw_rad))
    y_pixel_rotated = (x_pixel * np.sin(yaw_rad)) + (y_pixel * np.cos(yaw_rad))
    # Return the result  
    return x_pixel_rotated, y_pixel_rotated


# Define a function to perform a translation
def translate_pix(x_pixel_rotated, y_pixel_rotated, xpos, ypos, scale):
    # Apply a scaling and a translation
    x_pixel_translated = (x_pixel_rotated / scale) + xpos
    y_pixel_translated = (y_pixel_rotated / scale) + ypos
    # Return the result
    return x_pixel_translated, y_pixel_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(x_pixel, y_pixel, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    x_pixel_rotated, y_pixel_rotated = rotate_pix(x_pixel, y_pixel, yaw)
    # Apply translation
    x_pixel_translated, y_pixel_translated = translate_pix(x_pixel_rotated, y_pixel_rotated, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(x_pixel_translated), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(y_pixel_translated), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world


# Define a function to perform a perspective transform
# noinspection PyPep8Naming
def perspective_transform(img, src, dst):
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))  # keep same size as input image

    return warped


def locate_rock(points_x, points_y):
    if points_x.size > 0 and points_y.size > 0:
        return np.int_(np.mean(points_x)), np.int_(np.mean(points_y))
    else:
        return None, None


# Apply the above functions in succession and update the Rover state accordingly
# noinspection PyPep8Naming
def perception_step(Rover):
    # type: (RoverState) -> RoverState
    # Perform perception steps to update Rover()
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    dst_size = 5
    bottom_offset = 6
    source = np.float32([[14, 140], [301, 140], [200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1] / 2 - dst_size, Rover.img.shape[0] - bottom_offset],
                              [Rover.img.shape[1] / 2 + dst_size, Rover.img.shape[0] - bottom_offset],
                              [Rover.img.shape[1] / 2 + dst_size, Rover.img.shape[0] - 2 * dst_size - bottom_offset],
                              [Rover.img.shape[1] / 2 - dst_size, Rover.img.shape[0] - 2 * dst_size - bottom_offset],
                              ])

    # 2) Apply perspective transform
    birds_view = perspective_transform(Rover.img, source, destination)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    thresholded_terrain = color_threshold(birds_view, rgb_thresh=(160, 160, 160))
    thresholded_obstacles = obstacles_threshold(birds_view,
                                                threshold_high=(160, 160, 100),
                                                threshold_low=(0, 0, 0))
    thresholded_rocks = rocks_threshold(birds_view)

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.terrain = thresholded_terrain
    Rover.vision_image[:, :, 0] = thresholded_obstacles * 255
    Rover.vision_image[:, :, 1] = thresholded_rocks * 255
    Rover.vision_image[:, :, 2] = thresholded_terrain * 255

    # 5) Convert map image pixel values to rover-centric coords
    terrain_x_pixel, terrain_y_pixel = rover_coords(thresholded_terrain)
    obstacles_x_pixel, obstacles_y_pixel = rover_coords(thresholded_obstacles)
    rocks_x_pixel, rocks_y_pixel = rover_coords(thresholded_rocks)

    # 6) Convert rover-centric pixel values to world coordinates
    xpos, ypos, yaw = Rover.pos[0], Rover.pos[1], Rover.yaw
    world_size = Rover.worldmap.shape[0]
    scale = dst_size * 2

    terrain_x_world, terrain_y_world = pix_to_world(terrain_x_pixel, terrain_y_pixel,
                                                    xpos, ypos, yaw, world_size, scale)

    obstacles_x_world, obstacles_y_world = pix_to_world(obstacles_x_pixel, obstacles_y_pixel,
                                                        xpos, ypos, yaw, world_size, scale)

    rocks_x_world, rocks_y_world = pix_to_world(rocks_x_pixel, rocks_y_pixel, xpos, ypos, yaw, world_size, scale)

    rocks_x_world, rocks_y_world = locate_rock(rocks_x_world, rocks_y_world)

    # 7) Update Rover worldmap (to be displayed on right side of screen)
    if (Rover.roll <= 1.0 or Rover.roll >= 359.0) and (Rover.pitch <= 1.0 or Rover.pitch >= 359.0):
        Rover.worldmap[obstacles_y_world, obstacles_x_world, 0] += 1
        Rover.worldmap[rocks_y_world, rocks_x_world, 1] += 1
        Rover.worldmap[terrain_y_world, terrain_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(terrain_x_pixel, terrain_y_pixel)
    # Rover.obs_dists, Rover.obs_angles = to_polar_coords(obstacles_x_pixel, obstacles_y_pixel)
    Rover.rock_dists, Rover.rock_angles = to_polar_coords(rocks_x_pixel, rocks_y_pixel)
    # do not change the value unless you see a rock. it would be reset after collecting
    if not Rover.picking_up and len(Rover.rock_angles) > 0 and np.mean(Rover.rock_dists) <= 100:
        # pick up only what is in my way.
        Rover.seen_rock = Rover.seen_rock if rocks_x_world is None else (rocks_x_world, rocks_y_world)

    return Rover
