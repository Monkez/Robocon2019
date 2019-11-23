import numpy as np
import cv2
import threading
import time
import config as cf
import RPi.GPIO as GPIO

#LEDs
N = 9
M = 4

def move(direction, speed):
    if cf.pause:
        this_speed = 0
    else:
        this_speed = speed
    direction = -np.multiply(cf.cab, direction)
    message = "MOV"
    for i in direction:
        message+= " " + str(int(i*speed))
    message+= " /"
    cf.ser.write(message.encode())

    
def set_command(command, value):
    cf.command = (command, value)
    cf.pause = True
    
def adjust_car(direction):
    print("adjust_car", direction)
    cf.pause = False
    done = False
    opposite = (direction+2)%4
    while not done and not cf.pause:
        LEDs_weighted1 = cf.LEDs_weighted[cf.LEDs_map[direction]]
        LEDs_weighted2 = cf.LEDs_weighted[cf.LEDs_map[opposite]]
        center_led = int((N-1)/2)
        left_max = np.max(LEDs_weighted1[:center_led])
        right_max = np.max(LEDs_weighted1[center_led+1:])
        minus1 = np.sign(left_max - right_max)
        left_max = np.max(LEDs_weighted2[:center_led])
        right_max = np.max(LEDs_weighted2[center_led+1:])
        minus2 = np.sign(left_max- right_max)
        print(minus1, minus2)
        if minus1 == 0 and minus2==0:
            done = True
        else:
            vector1 = minus2*np.array([1, -1])
            vector2 = minus1*np.array([-1, 1])
            if minus1*minus2 >= 0:
                vector1 = - vector1
                vector2 = -vector2
            if direction == 0:
                vector = [vector1[0], vector1[1], vector2[0], vector2[1]]
            if direction == 1:
                vector = [vector2[1], vector1[0], vector1[1], vector2[0]]
            print(vector1, vector2, vector)
            if cf.pause:
                speed = 0
            else:
                speed = 100
            move(vector, speed)
            time.sleep(1)
    move(cf.PAUSE, 0)
    
def go_with_line(values):
    if len(values) == 1:
        direction = values[0]
        row_counter = (direction+1)%4
        num = 1000
    else:
        direction, row_counter, num = values
    print("go_with_line: ", direction, ",row couters", row_counter, ",num:", num)
    cf.pause = False
    while not cf.pause and num > 0:
        time.sleep(0.05)
        DIRECTION = cf.DIRECTIONs[direction]
        LEDs_MODE_direction = cf.LEDs_MODE[cf.LEDs_map[direction]]
        LEDs_weighted_direction = cf.LEDs_weighted[cf.LEDs_map[direction]]
        center_led = int((N-1)/2)
        check_led = LEDs_MODE_direction[center_led]
        total_weight = np.sum(LEDs_MODE_direction)
        left_max = np.max(LEDs_weighted_direction[:center_led])
        right_max = np.max(LEDs_weighted_direction[center_led+1:])
        minus = left_max - right_max
        if minus == 0 and check_led and total_weight!=0 and total_weight!=N:
            cf.rotate = False
            time.sleep(0.01)
        if minus != 0:
            cf.rotate = True
            cf.ratio = 1.5*minus
        if not cf.rotate:
            move(DIRECTION, cf.speed)
            time.sleep(0.001)
        else:
            UP_ratio = 2
            vector = (UP_ratio*np.array(DIRECTION)+cf.ratio*np.array(cf.TURN_LEFT))/(UP_ratio+abs(cf.ratio))
            move(vector, 130)
            time.sleep(0.01)       
        LEDs_MODE_counter = cf.LEDs_MODE[cf.LEDs_map[row_counter]]
        if LEDs_MODE_counter[center_led]:
            num -= 1
            if num>0:
                while LEDs_MODE_counter[center_led]:
                    move(DIRECTION, cf.speed)
                    time.sleep(0.1)
                    LEDs_MODE_counter = cf.LEDs_MODE[cf.LEDs_map[row_counter]]
    move(cf.PAUSE, 0)
    
def pause():
    cf.pause = True
    cf.command = ("pause", -1)
    move(cf.PAUSE, 0)
    print("pause")
    
def quit():
    cf.pause = True
    cf.RUN = False
    cf.command = ("quit", -1)
    move(cf.PAUSE, 0)
    print("quit")