import numpy as np
import cv2
import MAP
from HAL import HAL

# Window name
WINDOW_NAME = "NU GUI"

class GUI:
    """ Class to emulate unibotics GUI API """
    def __init__(self, robot=None):
        """ Read the map and initialize variables """
        self.map = MAP.getMap()
        self.particles = []
        self.laser = []
        self.resetGUI()
        self.robot = robot
        if self.robot is None:
            self.robot = HAL()

    def getRobotPose(self):
        return self.robot.pose

    def resetGUI(self):
        """ Reset the GUI image.
            Remove all particles and other lines and keep only the empty map
        """
        self.gui_map = cv2.cvtColor(self.map, cv2.COLOR_GRAY2RGB)
    
    def getImage(self):
        """ Returns the color image that is shown in the GUI.
            Use with caution!!
        """
        return self.gui_map
    
    def updateImage(self, new_gui_map):
        """ Update the image that is shown in the GUI. """
        self.gui_map = new_gui_map
        self.updateGUI()

    def updateGUI(self, block=False, show_particles=True, show_laser=True, wait_time=1):
        """ Take a "simulation" step and update the GUI elements:
            - Robot position
            - Particles
            Set "block" arg as True to block the simulation until a key is pressed.
            It is possible to enable/disable some drawing items with the corresponging show_* args
        """
        # Reset the map canvas
        self.resetGUI()

        # Add the list of particles (if any)
        if show_particles and len(self.particles) > 0:
           self.drawParticles()

        # Add laser readings
        if show_laser and len(self.laser) > 0:
            self.drawLaser()

        # Draw the robot's pose
        self.drawRobot(self.robot.pose)

        # Update the robot's pose after drawing
        self.robot.updatePose()

        cv2.imshow(WINDOW_NAME, self.gui_map)

        if block:
            wait_time = 0
        cv2.waitKey(wait_time)

    def drawRobot(self, pose, arrow_length=10, thickness=2, color=(0,0,255)):
        """ Draw the robot in the map.
            It is possible to change the color and size of the arrow marker.
            Pose is converted here from world to map coordinates.
        """
        map_pose = MAP.worldToMap(*pose)
        start_x = map_pose[0]
        start_y = map_pose[1]
        yaw = map_pose[2]
        end_x = int(start_x + arrow_length * np.cos(yaw))
        end_y = int(start_y + arrow_length * np.sin(yaw))
        cv2.arrowedLine(self.gui_map, (start_x, start_y), (end_x, end_y),
                        color=color, thickness=thickness, tipLength=0.3)

    def showParticles(self, particles):
        """ Store and prepare the list of particles to be drawn in the next update.
            Note: This function does not update the GUI. updateGUI must be called manually.
        """
        self.particles = particles

    def drawParticles(self, color=(255,0,0), arrow_length=10, thickness=1):
        """ Draw the particles in the map.
            Particles are expected in world coordinates (x, y, yaw) (m, m, rad).
            It is possible to change the color and size of the arrow markers.
        """
        for p_world in self.particles:
            self.drawRobot(p_world, arrow_length=arrow_length, thickness=thickness, color=(255,0,0))

    def showLaser(self, laser):
        """ Store and prepare the laser data to be drawn in the next update.
            Note: This function does not update the GUI. updateGUI must be called manually.
        """
        self.laser = laser

    def drawLaser(self, color=(0,0,255), point_size=2):
        """ Draw the laser end points in the map.
            Laser data is expected to be in world coordinates (x, y, yaw) (m, m, rad).
            It is possible to change the color and size of the arrow markers.
        """
        # Convert laser from world to map coordinates
        laser_cells = MAP.worldToMapArray(self.laser)
        # Iterate and draw a circle marker at each endpoint
        for laser_point in laser_cells:
            laser_point = int(laser_point[0]), int(laser_point[1])
            cv2.circle(self.gui_map, laser_point, radius=point_size, thickness=cv2.FILLED, color=color)

