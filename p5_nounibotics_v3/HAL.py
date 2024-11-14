import time
import numpy as np

import MAP

# Maximum laser detection distance in meters
MAX_LASER_DISTANCE = 100

# Value of obstacle cells in the occupancy grid map
OBSTACLE_VALUE = 0

class HAL:
    """ Hardware Abstraction Layer.
        This class provides funcitons to move the robot (setV/setW) and to read the laser sensor.
    """
    def __init__(self, initial_pos=MAP.ROBOT_START_POSITION):
        self.pose = initial_pos.copy()
        self.map_array = MAP.getMap()
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.last_update_time = time.time()

    def getPose(self):
        """ Returns the 2D pose as a tuple (x, y, yaw) """
        return self.pose

    def virtual_laser_beam(self, start_x, start_y, end_x, end_y):
        """ Generates a line using DDA algorithm until an obstacle is found
            or the end point is reached.
            Returns the point (x,y) where the line ends.
            If the end point is reached, returns infinite.
        """
        # Find the slope and direction of the line
        dx = int(abs(end_x - start_x))
        dy = int(abs(end_y - start_y))
        # Number of steps (dx or dy depending on what is bigger)
        steps = max(dx, dy)

        # Adjust dx and dy to the small step value according to previous calculations
        dx = (end_x - start_x) / steps
        dy = (end_y - start_y) / steps

        for i in range(0, steps):
            # Compute the indices for each step and convert to int to get cell positions
            x = start_x + int(dx * i)
            y = start_y + int(dy * i)
            if self.map_array[y, x] == OBSTACLE_VALUE:
                return (x, y)

        return (np.inf, np.inf)

    def getLaserData(self):
        """ Returns the measurements from the laser sensor.
            Returns a list of (x,y) points in global world coordinates.
        """
        # Get the robot pose in map coordinates as the origin of the laser 
        start_x, start_y, robot_yaw_map = MAP.worldToMap(*self.pose)
        # Convert max laser detection distance from meters to map cells
        laser_distance_cells = MAX_LASER_DISTANCE * MAP.MAP_SCALE
        virtual_laser_xy = []
        for beam_angle in range(180):
            # Actual beam's angle in map coordinates
            # Substract 90º to have the center aligned with the robot
            angle = robot_yaw_map + np.radians(beam_angle) - np.pi/2
            # Compute the theoretical (max) endpoint of the laser
            end_x = start_x + laser_distance_cells * np.cos(angle)
            end_y = start_y + laser_distance_cells * np.sin(angle)
            # Get the laser measurement
            laser_x, laser_y = self.virtual_laser_beam(start_x, start_y, end_x, end_y)
            virtual_laser_xy.append((laser_x, laser_y, 0))
        world_laser_xy = MAP.mapToWorldArray(np.array(virtual_laser_xy))
        return world_laser_xy

    def setV(self, linear_vel):
        """ Sets the linear velocity """
        self.linear_vel = linear_vel

    def setW(self, angular_vel):
        """ Sets the angular velocity """
        self.angular_vel = angular_vel
    
    def updatePose(self):
        """ Update the pose of the robot after dt seconds """
        # Get the time diference since the last update
        update_time = time.time()
        dt = update_time - self.last_update_time
        # Update robot position according to the set velocities
        self.pose[0] += dt * self.linear_vel * np.cos(self.pose[2])
        self.pose[1] += dt * self.linear_vel * np.sin(self.pose[2])
        self.pose[2] += dt * self.angular_vel

        self.last_update_time = update_time
