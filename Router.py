import config as cofig
from ActiveFunction import *
import time

#LEDs
N = 9
M = 4

router = [
    ("GO", ["UP", "TIME", 3]),
    ("GO", ["UP",  "NEXT", "LEFT"]),
    ("GO_WITH_LINE", ["UP", "LEFT", 2]),
    ("ADJ_CAR", 0)
    
]

DIRECTION = ["UP", "LEFT", "DOWN", "RIGHT"]

def run_router():
    print("run router")
    for step in router:
        if cf.pause:
            break
        print(step)
        command_type = step[0]
        if command_type == "GO":
            direction = DIRECTION.index(step[1, 0])
            if step[1, 1] == "TIME":
                move(cf.DIRECTIONs[direction])
                time.sleep(step[1, 2])
            if step[1, 1] == "NEXT":
                row_counter = DIRECTION.index(step[1, 2])
                done = False
                center_led = int((N-1)/2) 
                while not done :
                    if cf.pause:
                        break
                    move(DIRECTION, cf.speed)
                    time.sleep(0.1)               
                    LEDs_MODE_counter = cf.LEDs_MODE[cf.LEDs_map[row_counter]]
                    done = LEDs_MODE_counter[center_led]
                               
                               
            move(cf.PAUSE, 0)
                               
        if command_type == "GO_WITH_LINE": 
            direction = DIRECTION.index(step[1, 0])
            row_counter = DIRECTION.index(step[1, 1])
            num = DIRECTION.index(step[1, 2])
            go_with_line([direction, row_counter, num])  
            
        if command_type == "ADJ_CAR":
            adjust_car(step[1])                                       
                                                     
def DecodeCommand():
    print("DecodeCommand threading started!")
    while cf.RUN:
        time.sleep(0.1)
        if cf.command[0] == 'adj_car':
            adjust_car(cf.command[1])
           
        if cf.command[0] == 'go_with_line':
            go_with_line(cf.command[1])
            
        if cf.command[0] == 'run_router':
            run_router()
            
    print("DecodeCommand threading stopped!")

 