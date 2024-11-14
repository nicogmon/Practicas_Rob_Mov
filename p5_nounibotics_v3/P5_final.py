from HAL import HAL
import numpy as np
from GUI import GUI
import multiprocessing as mp
import MAP
import time
import math
import random


# Number of particles
N_PARTICLES  = 700
N_LASER_RAYS = 5

# Constant robot velocities
LINEAR_VEL = 0.5
ANGULAR_VEL = 0.8


# Maximum laser detection distance in meters
MAX_LASER_DISTANCE = 100

# Value of obstacle cells in the occupancy grid map
OBSTACLE_VALUE = 0

# Time of the last propagation of the particles
last_update_time = time.time()
num_processes = mp.cpu_count()
robot = HAL()


def initialize_particles():
    """ Generate random particles in world coordinates (meters).
        X/Y values are constrained within the map limits.
        Yaw values are in the [0, 2*pi] range.
    """
    
    mapa = MAP.getMap()
    # Allocate space
    particles = []
    # Get the limits from the MAP module
    x_low, y_low = MAP.WORLD_LIMITS_LOW
    x_high, y_high = MAP.WORLD_LIMITS_HIGH
    # Distribute randomly in the map
    for i in range(N_PARTICLES):
        
        particle = np.random.uniform(low=[x_low, y_low, 0.0], high=[x_high, y_high, 2*np.pi], size=(3,))
        map_particle = MAP.worldToMap(particle[0], particle[1], particle[2])
            
        while(mapa[map_particle[1]][map_particle[0]] == 0):

            particle = np.random.uniform(low=[x_low, y_low, 0.0],
                                  high=[x_high, y_high, 2*np.pi],
                                  size=(3,))
            map_particle = MAP.worldToMap(particle[0], particle[1], particle[2])
            
        particles.append([particle[0], particle[1], particle[2]])

    return np.array(particles)

def update_particle_pose(particle, dt):
    """ Update the pose of a particle in the dt period.
        Add a random Gaussian noise to the movement.
    """
    yaw = particle[2]
    # Estimate robot movement in dt according to the set velocities
    dx = dt * LINEAR_VEL * np.cos(yaw)
    dy = dt * LINEAR_VEL * np.sin(yaw)
    dyaw = dt * ANGULAR_VEL
    # Add this movement to the particle, with an extra Gaussian noise
    particle[0] += dx + np.random.normal(0.0, 0.02)
    particle[1] += dy + np.random.normal(0.0, 0.02)
    particle[2] += dyaw + np.random.normal(0.0, 0.01)

def propagate_particles(particles):
    """ Estimate the movement of the robot since the last update
        and propagate the pose of all particles according to this movement.
    """
    global last_update_time
    # Get the time diference since the last update
    current_time = time.time()
    dt = current_time - last_update_time
    # Update all particles according to dt
    for p in particles:
        update_particle_pose(p, dt)
    # Reset the update time
    last_update_time = current_time
    return particles


def compute_particle_weights(gui_obj,particles):
    """ Compute the weight of each particle.
        This function should generate a virtual laser measurement
        for each particle and compute the error (difference)
        between the virtual laser and the actual sensor measurement.

        This example function just generates silly weights.
    """
    mapa = MAP.getMap()
    laser_ref = HAL.getLaserData(robot)
    gui_obj.showLaser(laser_ref)
    
    
    #nav(laser_ref)
   
    map_particles = []
    for particle in particles:
        map_particle = MAP.worldToMap(particle[0], particle[1], particle[2])

       
        
        if map_particle[0] < 0 or map_particle[1] < 0 or map_particle[0] >= MAP.MAP_WIDTH or map_particle[1] >= MAP.MAP_HEIGHT:
            if map_particle[0] < 0:
                map_particle = (0, map_particle[1], map_particle[2])
            if map_particle[1] < 0:
                map_particle = (map_particle[0], 0, map_particle[2])
            if map_particle[0] >= MAP.MAP_WIDTH:
                map_particle = (MAP.MAP_WIDTH - 1, map_particle[1], map_particle[2])
            if map_particle[1] >= MAP.MAP_HEIGHT:
                map_particle = (map_particle[0], MAP.MAP_HEIGHT - 1, map_particle[2])
            map_particles.append(map_particle)
            continue
                

        map_particles.append(map_particle)
        
    #convertir a mapa las coordenadas 
    pool = mp.Pool(processes=num_processes)
    lasers = pool.map(get_virtual_laser,map_particles)
    weights = []
    
    for laser in lasers:
        #convertir a mapa las coordenadas 
        for i in range(len(laser)):
            
            error = np.sum(np.abs(laser[i] - laser_ref[i*N_LASER_RAYS]))
            
            if error == 0:
                error = 0.000001
        weights.append(1/error)
    
    

    return weights

        
#intento de hacer un segundo uso de multiprocessing para la comparacion de los laser
#dificultad para pasar como argumento ambos laseres.
def compare_laser(laser):
    error = 0
        #convertir a mapa las coordenadas
    
    for i in range((len(laser)-180)):
            
        error = np.sum(np.abs(laser[i] - laser[(len(laser)-180+(i*N_LASER_RAYS))]))
            
        if error == 0:
            error = 0.000001
   

    return 1/error




def resample_particles(old_particles, weights):
    """ Resample the set of particles given their weights. """
    # Allocate space for new particles
    particles = np.zeros((N_PARTICLES, 3))

    # Normalize the weights so the total sum is 1
    weights /= np.sum(weights)

    # Get random indices from the list of particles
    selected_idx = np.random.choice(N_PARTICLES, replace=True, size=N_PARTICLES, p=weights)

    particles = old_particles[selected_idx]
    return particles

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

def get_virtual_laser(particle):
        """ Returns the measurements from the laser sensor.
            Returns a list of (x,y) points in global world coordinates.
        """
        start_x = particle[0]
        start_y = particle[1]
        start_yaw = particle[2]

        # Get the robot pose in map coordinates as the origin of the laser 
        
        # Convert max laser detection distance from meters to map cells
        laser_distance_cells = MAX_LASER_DISTANCE * MAP.MAP_SCALE
        virtual_laser_xy = []
        for beam_angle in range(180):#para reducir tiempo coger un rayo de cada 3-5
            # Actual beam's angle in map coordinates
            # Substract 90º to have the center aligned with the robot
            if beam_angle % N_LASER_RAYS == 0:
                angle = start_yaw + np.radians(beam_angle) - np.pi/2
                # Compute the theoretical (max) endpoint of the laser
                end_x = start_x + laser_distance_cells * np.cos(angle)
                end_y = start_y + laser_distance_cells * np.sin(angle)
                # Get the laser measurement
                laser_x, laser_y = virtual_laser_beam(robot, start_x, start_y, end_x, end_y)
                virtual_laser_xy.append((laser_x, laser_y, 0))
        world_laser_xy = MAP.mapToWorldArray(np.array(virtual_laser_xy))
        return world_laser_xy

def nav(laser_data):
    for i in range(len(laser_data)):
        
        dist = math.sqrt(laser_data[i][0]**2 + laser_data[i][1]**2)
        print (dist)
        if dist < 1.5:
            robot.setV(0)
            robot.setW(1.2)
            return
    robot.setV(LINEAR_VEL)
    robot.setW(0)

def main():
    
    # Set a custom initial pose
    robot.pose[0] = 0.5

    # Create a GUI object and link it with the robot
    gui = GUI(robot=robot)
    

    # Initialize some random particles
    particles = initialize_particles()
    gui.showParticles(particles)

    # Robot pose is automatically updated inside the GUI "updateGUI"
    gui.updateGUI()
    
    # Set a small velocity
    robot.setV(LINEAR_VEL)
    robot.setW(ANGULAR_VEL)
    
    # Store the time of the last pose update
    last_update_time = time.time()
    
    while True:
        # Propagation (prediction) step
        # Show the particles in the GUI
        gui.showParticles(particles)

        # Robot pose is automatically updated inside the GUI "updateGUI"
        gui.updateGUI()
        
        weights = []
        particles = propagate_particles(particles)

        # Compute the weights
        weights = compute_particle_weights(gui,particles)
        

        # Resample the particles
        particles = resample_particles(particles, weights)
        
        
        
        
        
if __name__ == '__main__':
    main()
