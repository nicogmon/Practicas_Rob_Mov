from GUI import GUI
from HAL import HAL
import math
import numpy as np
# Enter sequential code!

CFFactor_x = 0.7 
CFFactor_y = 0.2
OFFactor_x = 11.5
OFFactor_y  = 20
MAX_DISTANCE = 30
MAX_ATTRAC_FX = 5
MAX_ATTRAC_FY = 3
REACH_DISTANCE = 2


def absolute2relative (x_abs, y_abs, robotx, roboty, robott):

    # robotx, roboty are the absolute coordinates of the robot
    # robott is its absolute orientation
    # Convert to relatives
    dx = x_abs - robotx
    dy = y_abs - roboty

    # Rotate with current angle
    x_rel = dx * math.cos (-robott) - dy * math.sin (-robott)
    y_rel = dx * math.sin (-robott) + dy * math.cos (-robott)

    return x_rel, y_rel 
    
    
def parse_laser_data (laser_data):
    laser = []
    i = 0
    while (i < 180):
        dist = laser_data.values[i]
        if dist > 10:
            dist = 10
        angle = math.radians(i-90) # because the front of the robot is -90 degrees
        laser += [(dist, angle)]
        i+=1
    return laser
    

def laser_vector(laser):
    laser_vectorized = []
    for d,a in laser:
        # (4.2.1) laser into GUI reference system
        x = np.exp(-d) * math.cos(a) * -1
        y = np.exp(-d) * math.sin(a) * -1
        v = (x,y)
        laser_vectorized += [v]

    laser_mean = np.mean(laser_vectorized, axis=0)
    return laser_mean
    
    
    
while True:
    GUI.showImage(HAL.getImage())

    #Obtencion posicion del objetivo
    currentTarget = GUI.map.getNextTarget()

    #obtencion posicion del robot
    robotx = HAL.getPose3d().x
    roboty = HAL.getPose3d().y
    robott = HAL.getPose3d().yaw
    target_abs_x = currentTarget.getPose().x
    target_abs_y = currentTarget.getPose().y
    
    #vector de atraccion
    local_target = absolute2relative(target_abs_x, target_abs_y, robotx, roboty, robott)
    
    #vector de repulsion
    laser_mean = HAL.getLaserData()
    laser = parse_laser_data(laser_mean)
    laser_vectorized = laser_vector(laser)
    
    carForcex = local_target[0]
    carForcey = local_target[1]
    
    distance = math.sqrt((carForcex * carForcex) + (carForcey * carForcey))
    carForceAng = math.atan(carForcey/carForcex)
    
    
    if distance < REACH_DISTANCE:
      currentTarget.setReached(True)
    # Current target
    GUI.showLocalTarget(local_target)
    
    
    
    if distance > MAX_DISTANCE:
      carForcex = MAX_ATTRAC_FX
      carForcey = 0
            
    if carForcex > MAX_ATTRAC_FX:
      carForcex = MAX_ATTRAC_FX
    if carForcey > MAX_ATTRAC_FY:
      carForcey = 1
      
     # Car direction (green line in the image below)
    carForce = [CFFactor_x * carForcex, CFFactor_y * carForcey]
    
    # Obstacles direction (red line in the image below)
    obsForce = [OFFactor_x * laser_vectorized[0], OFFactor_y * laser_vectorized[1]]
  
    avgForce = [1 + carForce[0] + obsForce[0], carForce[1] + obsForce[1]]
    
    if obsForce[0] < -0.9:
      print("OBJETO MUY CERCA")
      avgForce[0] = avgForce[0]/2
      
      
    if avgForce[0] < 0.2:
      avgForce = [ 1 + avgForce[0] ,carForce[1] +  2 * obsForce[1]]
      print("poca fuerza")
      
    HAL.setV(avgForce[0])
    HAL.setW(avgForce[1])
  
  
    GUI.showForces(carForce, obsForce, avgForce)
    
    
