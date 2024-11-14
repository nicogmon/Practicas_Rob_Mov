from GUI import GUI
from HAL import HAL
import math
import time 
import numpy as np
import sys
import random
# Enter sequential code!


def parse_laser_data(laser_data):
    laser = []
    for i in range(180):
        dist = laser_data.values[i]
        angle = math.radians(i)
        laser += [(dist, angle)]
    return laser
    

    
    
def spiral(data):
  inic_time = time.time()
  lin_vel = 0.3
  ang_vel = 1.7
  while(HAL.getBumperData().state != 1 and not obstacle(data)):
    if time.time() - inic_time > 24:
      return 1
    data = parse_laser_data(HAL.getLaserData())
    HAL.setV(lin_vel)
    HAL.setW(ang_vel)
    lin_vel += 0.07
    
    time.sleep(0.7)
    
  return 1

state = 1
FORWARD = 0
SPIRAL= 1 
TURN = 2
BACK = 4

list = [-1, 1,1,1,1,-1]
turn_list = [2, 4, 6, 8]
start_time = 0
direction = 1

started = 0
last_state = 3

def obstacle (data):
  for i in range(60):
    if data[60+i][0] < 0.35:
      print(data[60+i][0])
      return 1
  return 0
    
  

while True:
    data = parse_laser_data(HAL.getLaserData())
    
    phase = 0
    if state == FORWARD:
      HAL.setV(2.5)
      HAL.setW(0) 
      if (start_time == 0):
      	start_time = time.time()
      	#print("TIEMPO ACTUALIZADO")
      	
      if obstacle(data) == 1: 
        state = BACK
        
        if time.time() - start_time > 25  :
          last_state = FORWARD
        #print("LASEEEEER")
        
      
      elif time.time() - start_time > random.randint(3,8) and data[90][0] > 1:
        state = SPIRAL
        start_time = 0
        
    elif state == BACK:
      HAL.setV(-1)
      HAL.setW(0)
      #print("BACK")
      
      if obstacle(data) == 0:
        if last_state == FORWARD:
          state = SPIRAL
        else:
          state = TURN
      
      start_time = 0
      last_state = 3
      
      
    elif state == TURN:
      #print("TURN")
      
      if start_time == 0:
      	start_time = time.time()
      	direction = random.choice(list)
      	#print("TIEMPO ACTUAL 00000")
      	print(direction)
        
      HAL.setV(0)
      #HAL.setW(0.9 * direction)
      HAL.setW(1.2 * direction)
      if time.time() - start_time > random.randint(2,5):
        #print("FFIN GIROOOOOO")
        HAL.setW(0)
        start_time = 0
        state = FORWARD
      if obstacle(data): 
        start_time = 0
        state = BACK
    
    elif state == SPIRAL:
      spiral(data)
      state = BACK
      start_time = 0
      
    else:
      print("unknown state")
