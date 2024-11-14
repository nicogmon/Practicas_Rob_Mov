""" This module contains the map utility functions to
    read the map image and convert between world and map coordinates.
    
    The Map coordinate system follows OpenCV coordinates:
        0------> X
        |
        |
        y
    
    The World coordinate system is similar to the gazebo world in Unibotics:
        --------------
        |             |
        |             |
        | x<---0      |
        |      |      |
        |      |      |
        |      y      |
        --------------

    Several global variables are available with info related to map size, scale and limits
"""

import cv2
import numpy as np

# Path to map image
MAP_FILE = "mapgrannyannie.png"

# x, y, yaw in world coordinate system (m, m, rads): x left, y bottom
ROBOT_START_POSITION = [3.5, 1.0, -np.pi/2]


# MAP_WIDTH = 1013
# MAP_HEIGHT = 1012
# Resized image
MAP_WIDTH = 400
MAP_HEIGHT = 400


# This is a fake "invented" scale
# We will assume that the map is a 10x10m area
# MAP_SCALE = 100 # Original image
MAP_SCALE = 40 # Resized image

# Map coordinates for origin position in the real world (center of apartment)
# These are the map coordinates of the world's (0,0)
# MAP_OFFSET = (500, 650) # Original image
MAP_OFFSET = (200, 260) # Resized image

# Map limits in world coordinates. Numbers outside these ranges will be out of the matrix bounds.
# For 1013x1012 image
# WORLD_LIMITS_LOW = (-5.12, -6.5)
# WORLD_LIMITS_HIGH = (5.0, 3.61)
# For 400x400 image
WORLD_LIMITS_LOW = (-4.975, -6.5)
WORLD_LIMITS_HIGH = (5.0, 3.475)

def getMap():
    """ Read the map image as a grayscale numpy array """
    map_img = cv2.imread(MAP_FILE, cv2.IMREAD_GRAYSCALE)
    map_img = cv2.resize(map_img, dsize=(MAP_WIDTH, MAP_HEIGHT), interpolation=cv2.INTER_NEAREST)
    return map_img

def mapToWorld(mx, my, myaw=0.0):
    """ Convert map coordinates (pixels) to real world coordinates (meters) """
    wx = - (mx - MAP_OFFSET[0]) / MAP_SCALE
    wy = (my - MAP_OFFSET[1]) / MAP_SCALE
    wyaw = (-myaw + np.pi) % (2*np.pi)
    return wx, wy, wyaw

def worldToMap(wx, wy, wyaw=0.0):
    """ Convert real world coordinates (meters) to map coordinates (pixels) """
    mx = int(-MAP_SCALE * wx + MAP_OFFSET[0])
    my = int(MAP_SCALE * wy + MAP_OFFSET[1])
    myaw = (-wyaw + np.pi) % (2*np.pi)
    return mx, my, myaw

def mapToWorldArray(map_coords):
    """ Convert an array of map coordinates (pixels) to real world coordinates (meters) """
    scale = np.array([-MAP_SCALE, MAP_SCALE])
    world_coords = np.empty(map_coords.shape)
    world_coords[:, :2] = (map_coords[:, :2] - MAP_OFFSET) / scale
    world_coords[:, 2] = np.mod(-map_coords[:, 2] + np.pi, 2*np.pi)
    return world_coords

def worldToMapArray(world_coords):
    """ Convert an array of real world coordinates (meters) to map coordinates (pixels) """
    offset = np.array([MAP_OFFSET[0], MAP_OFFSET[1]])
    scale = np.array([-MAP_SCALE, MAP_SCALE])
    map_coords = np.empty(world_coords.shape)
    map_coords[:, :2] = scale * world_coords[:, :2] + offset
    map_coords[:, 2] = np.mod(-world_coords[:, 2] + np.pi, 2*np.pi)
    return map_coords


if __name__ == '__main__':
    # Testing conversion functions
    world_p = (2.4, -3.2, np.pi/3)
    map_p = worldToMap(*world_p)
    print(F"World pose: {world_p}")
    print(F"Map pose: {map_p}")

    world_p_2 = mapToWorld(*map_p)
    print(F"Reconverted world pose: {world_p_2}")

    # Testing array conversion functions
    world_poses = np.array([
        (2.4, -3.2, np.pi/3),
        (0.0, 0.0, np.pi),
    ])
    map_poses = worldToMapArray(world_poses)
    print(F"World pose array:\n{world_poses}")
    print(F"Map pose array:\n{map_poses}")

    world_poses_2 = mapToWorldArray(map_poses)
    print(F"Reconverted world pose array:\n{world_poses_2}")

    # Check map size and limit coordinates
    map_array = getMap()
    print(F"Map size:{map_array.shape}")
    print(F"Lowest map limits (in world coords): {mapToWorld(0, 0)}")
    print(F"Highest map limits (in world coords): {mapToWorld(MAP_WIDTH - 1, MAP_HEIGHT - 1)}")

    # Verification of map limits
    map_limit_low = worldToMap(*WORLD_LIMITS_LOW)
    print(F"Map limit low: {map_limit_low}")
    print(F"Map value at limit low: {map_array[map_limit_low[1], map_limit_low[0]]}")

    map_limit_high = worldToMap(*WORLD_LIMITS_HIGH)
    print(F"Map limit high: {map_limit_high}")
    print(F"Map value at limit high: {map_array[map_limit_high[1], map_limit_high[0]]}")
