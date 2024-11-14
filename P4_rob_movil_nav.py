from GUI import GUI
from HAL import HAL
import math
import numpy as np
from MAP import MAP
import cv2
import queue
import time
import copy
# Enter sequential code!

MAP_WIDTH = 400
MAP_HEIGHT= 400

CAR_POSITION = tuple(MAP.rowColumn((HAL.getPose3d().x , HAL.getPose3d().y)))

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

def normalize_grid(grid):
    mod_grid = copy.deepcopy(grid)
    walls = []
    for i in range(MAP_WIDTH):
      for j in range(MAP_HEIGHT):
        if mod_grid[i][j] == 1000:
            mod_grid[i][j] = 0
            walls.append((i,j))
  
    max_grid = np.max(grid)
    if max_grid == 0:
      max_grid = 1
    mod_grid = np.clip(mod_grid * 255 / max_grid, 0, 255).astype('uint8')
    
    for wall in walls:
      mod_grid[wall[0]][wall[1]] = 255
    
    return mod_grid
    
    
def gridToWorld(map_cell):
  world_x = map_cell[1] * 500 / 400 - 250
  world_y = map_cell[0] * 500 / 400 - 250
  
  return tuple((world_x, world_y))
  
  

def expand_node(node, map_img, grid, priorityQueue, obstacles):
    # This function expands a node in the grid
    # and returns the list of expanded nodes
    # and the cost of the expanded nodes
    # The cost of the expanded nodes is the same
    # as the cost of the node + 1 or sqrt of 2 in diagonal nodes

    for i in range(-1, 2):
        if node[1][0] + i < 0 or node[1][0] + i >= MAP_WIDTH:
            continue
        for j in range(-1, 2):
            if node[1][1] + j < 0 or node[1][1] + j >= MAP_HEIGHT:
              continue
            if i == 0 and j == 0:
              continue

            if grid[node[1][0] + i][node[1][1] + j] == 0:
                add_cost = 1
                if j != 0 and i != 0:
                    add_cost = 1.9
                if map_img[node[1][0] + i][node[1][1] + j] != 0:
                    grid[node[1][0] + i][node[1][1] + j] = node[0] + add_cost
                    priorityQueue.put((node[0] + add_cost, (node[1][0] + i, node[1][1] + j)))
                    
                else:
                    obstacles.append((node[1][0] + i,node[1][1] + j))
                    grid[node[1][0] + i][node[1][1] + j] = 1000
                    
                  
    return priorityQueue
    
def add_objects(grid,map_array):
  for i in range(400):
    for j in range(400):
      if map_array[i][j] == 0:
        grid[i][j] = 1000
    
    
def find_path(new_target_Map, grid,obstacles):
  cost = 500
  path = []
  old_node = tuple(MAP.rowColumn((round(HAL.getPose3d().x) , round(HAL.getPose3d().y))))
  while cost > 1:
     
        node = nearest_node(grid , old_node, obstacles)
        cost = grid[node[0]][node[1]]
        path.append([node[1],node[0]])
        
        old_node = node
      
        time.sleep(0.2)
  print(path)
  return path
  
  
  
  
def expand_walls(grid, obstacles,expanded_walls, new_walls):
  
  for node in obstacles:
    for i in range(-1,2):
      if node[0] + i < 0 or node[0] + i >= MAP_WIDTH:
            continue
      for j in range(-1, 2):
          if node[1] + j < 0 or node[1] + j >= MAP_HEIGHT:
              continue
          if i == 0 and j == 0:
              continue
          if (node[0] + i,node[1] + j) not in expanded_walls:
              grid[node[0] + i][node[1] + j] += 50
              expanded_walls.append((node[0] + i,node[1] + j))
  factor_inic = 40
      
  for i in range(3):
    already_exp = copy.deepcopy(expanded_walls)
    
    print("Expandiendo paredes...")
    for node in already_exp:
      for i in range(-1,2):
        if node[0] + i < 0 or node[0] + i >= MAP_WIDTH-1:
            continue
        for j in range(-1, 2):
            if node[1]+ j < 0 or node[1] + j >= MAP_HEIGHT-1:
                continue
            if i == 0 and j == 0:
                continue
            if (node[0] + i,node[1] + j) not in expanded_walls:
                grid[node[0] + i][node[1] + j] += 10
                expanded_walls.append((node[0] + i,node[1] + j))
              
    factor_inic -= 10          


def nearest_node(grid,car_position,obstacles):
  node = car_position
  low_cost = grid[node[0],node[1]]
  nearest = node
  
  for i in range(-5, 6):
    
      if node[0] + i < 0 or node[0] + i >= MAP_WIDTH:
          continue
      for j in range(-5, 6):
          if -5 < i < 5 and -5 < j < 5:
              continue
          if node[1] + j < 0 or node[1] + j >= MAP_HEIGHT:
              continue
            
          node_cost = grid[node[0] + i ,node[1] + j]
          
          if node_cost == 0 or node in obstacles:
            continue
          
          if node_cost < low_cost:
            nearest =  tuple((node[0] + i ,node[1] + j))
            low_cost = node_cost
            
  return nearest


def nav(dest_node):

  dest_node = gridToWorld(dest_node)
  
  car_position = tuple((round(HAL.getPose3d().x) , round(HAL.getPose3d().y)))
  
  local_target_vector = absolute2relative(dest_node[0], dest_node[1], car_position[0], car_position[1], HAL.getPose3d().yaw)
  
  carForcex = local_target_vector[0]
  carForcey = local_target_vector[1]
  
  
  avgForce = [carForcex , carForcey]
  """if abs(avgForce[0]) > 5:
    avgForce[0] = 5
  if abs(avgForce[1]) < 2:
    avgForce[1] = 1"""
  
  HAL.setV(avgForce[0])
  HAL.setW(avgForce[1])
  
  
  return 0
    
map_img = MAP.getMap('/RoboticsAcademy/exercises/static/exercises/global_navigation_newmanager/resources/images/cityLargenBin.png')
target = None 
extra_rounds = 0
flag = 0


while True:
    # Enter iterative code!
    # Get the map image and show the car and target positions
    
    new_target = GUI.getTargetPose()
    new_target_Map =tuple(MAP.rowColumn(new_target))
    
    
    if new_target != target:
      target = new_target 
    
      priorityQueue = queue.PriorityQueue()
      priorityQueue.put((0, (new_target_Map[1], new_target_Map[0])))
      expanded = []
      obstacles = []

      grid = np.zeros((MAP_WIDTH, MAP_HEIGHT))
      print("Creando grid")
      while not priorityQueue.empty():
        
        if flag == 1:
          extra_rounds += 1
          if extra_rounds == 5000:
            GUI.showNumpy(grid)
            break
      
        node = priorityQueue.get()

        if node[1] == (CAR_POSITION[1],CAR_POSITION[0]):
            print("Posicion del coche alcanzada")
            flag = 1
        if node not in expanded:
            expanded.append(node)
            expand_node(node, map_img, grid, priorityQueue, obstacles)
 
      # Normalize the grid to show 
      grid_normalized = normalize_grid(grid)

      expanded_walls = []
      new_walls = []
      expand_walls(grid,obstacles,expanded_walls, new_walls)
      
      add_objects(grid, map_img)
      
      GUI.showNumpy(grid_normalized)
      
      car_position = CAR_POSITION 
      
      distance = 0
      
      car_position = tuple(MAP.rowColumn((round(HAL.getPose3d().x) , round(HAL.getPose3d().y))))
 
      final_dist = 200
      old_node = car_position
      while final_dist > 2:
        
        car_position = tuple(MAP.rowColumn((round(HAL.getPose3d().x) , round(HAL.getPose3d().y))))

        car_position = car_position[::-1]
        node = nearest_node(grid ,car_position,obstacles)
        node = node[::-1]
        
        nav(node)
        car_position = car_position[::-1]
        
        
        final_vect = (abs(car_position[0] - new_target_Map[0]),abs(car_position[1] - new_target_Map[1]))

        final_dist =  math.sqrt((final_vect[0] * final_vect[0]) + (final_vect[1]*final_vect[1]))

        time.sleep(0.1)
        
      print("terminado")
      HAL.setV(0)
      HAL.setW(0)

    exit()