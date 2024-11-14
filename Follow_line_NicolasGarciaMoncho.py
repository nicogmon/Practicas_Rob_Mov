from GUI import GUI
from HAL import HAL
import numpy as np
import argparse
import cv2
import time 

# Enter sequential code!


# load the image
Kp = 0.0093

Kd = 0.001

Ki = 0.008

# Valor anterior del error
prev_error = 0.0
error_acum = 0.0
centro = 0
centro2 = 0
sin_linea_count = 0
w = 3
# Tiempo anterior
prev_time = time.time()

while True:
   
    while True:
      
      if sin_linea_count == 5:
        sin_linea_count = 0
      image = HAL.getImage()
      
      
      pixels_count = 0
      pixels_count_turn = 0
      white_pixels = []
      white_pixels_turn = []
      
      # create NumPy arrays from the boundaries
      lower = np.array([17, 15, 100], dtype = "uint8")
      upper = np.array([50, 56, 255], dtype = "uint8")
      # find the colors within the specified boundaries and apply
      # the mask
      mask = cv2.inRange(image, lower, upper)
      output = cv2.bitwise_and(image, image, mask = mask)
      
      output[mask == 255] = [255, 255, 255]
      
      # show the images
      for i in range(0, 640):
        if (output[250][i] == [255, 255, 255]).all():
          pixels_count += 1
          output[250][i] = [255,255,0]
          white_pixels.append(i)
      posicion = pixels_count//2
      
      
      for i in range(0, 640):
        if (output[245][i] == [255, 255, 255]).all():
          output[245][i] = [0,255,0]
          pixels_count_turn += 1
          white_pixels_turn.append(i)
      posicion2 = pixels_count_turn//2
      
      
      GUI.showImage(output)
      
      
      if pixels_count == 0:
        sin_linea_count += 1
    
        
      if pixels_count != 0:
        centro = white_pixels[posicion]
      if pixels_count_turn != 0:
        centro2 = white_pixels_turn[posicion2]
      
      if centro == 0 or sin_linea_count == 5:
        
        HAL.setV(0)
        if w != 0:
          HAL.setW(w)
        else:
          HAL.setW(4)
        continue
      
      error = 320 - centro
      error2 = 320 - centro2 
 
      if error2 > 35 or error2 < -35:#Estado de curva 
        Kp = 0.0092

        Kd = 0.0035

        Ki = 0.008
        HAL.setV(5)
      else:
        HAL.setV(7)
        
      current_time = time.time()
      delta_error = error - prev_error
      delta_time = current_time - prev_time
      
      derivative = Kd * (delta_error / delta_time)
      
      
      if derivative > 15 :
        derivative = 15
      elif derivative < -15:
        derivative = -15
        
      error_acum +=error_acum * (delta_time)
      
      w = Kp * error + derivative + Ki * error_acum
      
      HAL.setW(w)
     
      prev_error = error
      prev_time = current_time
