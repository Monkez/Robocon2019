#! /usr/bin/env python3
import numpy as np
import cv2
import serial
import config as cf
import threading
import time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)  
cf.led6 = [23, 21, 19, 13, 15]
GPIO.setup(cf.led6, GPIO.OUT)
for led in cf.led6:
    GPIO.output(led, False)
    
font = cv2.FONT_HERSHEY_SIMPLEX
WIDTH = 400
HEIGHT = 400
cf.global_img = np.zeros((WIDTH, HEIGHT, 3), np.uint8)
cf.message_arr = ["NoMessage"]
cf.message = ""
cf.ser = [None, None]
stt = 0
for i in range(5):
    try:
        port = '/dev/ttyUSB'+str(i)
        cf.ser[stt] = serial.Serial(port, 250000)
        stt+=1
        print("Connected to arduino:", str(i))
        if stt ==2:
            break
    except:
        pass
    
t = time.time()
cf.message =""

cf.wait = True
def check():
    print("check_begin!")
    while cf.wait:
        try:
            cf.message = str(cf.ser[0].readline().decode())[:3]
            #print(cf.message)
            if cf.message == "UNO" or cf.message == "LED":
                break
        except:
            pass
check_threading = threading.Thread(target=check)
check_threading.start()
time.sleep(2)
for i in range(10):
    _  = cf.ser[0].write("WHO /".encode())
    time.sleep(0.2)
time.sleep(0.3)
cf.wait = False
if cf.message[0:3] == "UNO":
    cf.uno = cf.ser[0]
    cf.mega = cf.ser[1]
else:
    cf.uno = cf.ser[1]
    cf.mega = cf.ser[0]
    
def ping_to_arduino():
    print("=>> Ping to arduino...")
    message = "PUSH /"
    t1 = time.time()
    try:
        cf.mega.write(message.encode())
    except:
        pass
    while cf.message_arr[0] != 'PULL' and (time.time() - t1)<5:
        time.sleep(0.001)
    print("=>> Ping result:", time.time() - t1)

# LEDs parameters
cf.LED_ROWS = 4
cf.LED_COLLUMS = 9
cf.LEDs = np.zeros((cf.LED_ROWS , cf.LED_COLLUMS))
cf.LEDs_threshold = np.ones((cf.LED_ROWS , cf.LED_COLLUMS))
cf.LEDs_min = np.zeros((cf.LED_ROWS , cf.LED_COLLUMS))
cf.LEDs_max = np.zeros((cf.LED_ROWS , cf.LED_COLLUMS))
cf.LEDs_MODE = np.zeros_like(cf.LEDs)
cf.LEDs_MODE_show = np.zeros_like(cf.LEDs)
cf.LEDs_weighted = np.zeros_like(cf.LEDs_MODE)
cf.CENTER_LEDs_MODE = np.zeros(cf.LED_ROWS)
cf.MAX_LEFT_LEDs = np.zeros(cf.LED_ROWS)
cf.MAX_RIGHT_LEDs = np.zeros(cf.LED_ROWS)
cf.LEDs_threshold = np.load("/home/pi/Desktop/Git/Robocon2019/LEDs_threshold.npy")

#CTHTs
CTHTs = [31, 29, 35, 33, 37]
GPIO.setup(CTHTs, GPIO.IN)
cf.CTHTs = np.zeros(len(CTHTs))
cf.Counter = np.zeros(4)
cf.Counter_Mode = np.zeros((4, 10))
cf.Counter_e = [True, True, True, True]

cf.total_message = 0
cf.error_message = 0
cf.error_probality = -1

def read_inputs():
    print("** read_inputs is running...")
    t_counter = time.time()
    while cf.RUN:
        try:
            # LEDS
            cf.message = cf.mega.readline().decode()[:-2] 
            cf.message_arr = cf.message.split(" ")
            cf.total_message+=1
            if cf.total_message == 1000:
                cf.error_probality =  cf.error_message/ cf.total_message
                cf.total_message = 0
                cf.error_message = 0
            if cf.message_arr[0] == 'LEDs':
                LEDs_arr = np.array(cf.message_arr[1:].copy()).astype(np.int32)
                LEDs_arr = LEDs_arr.reshape((cf.LED_ROWS, cf.LED_COLLUMS))
                cf.LEDs = LEDs_arr.copy()
                LEDs_MODE = np.zeros_like(cf.LEDs)
                LEDs_MODE[cf.LEDs >= cf.LEDs_threshold] = 1
                LEDs_MODE[cf.LEDs < cf.LEDs_threshold] = 0
                #LEDs_MODE[0] = np.flip(LEDs_MODE[0].copy())
                #LEDs_MODE[3] = np.flip(LEDs_MODE[3].copy())
                #LEDs_MODE[2] = np.flip(LEDs_MODE[2].copy())
                cf.LEDs_MODE_show = LEDs_MODE.copy()
#                 cf.LEDs_MODE_show[2] = np.flip(cf.LEDs_MODE_show[2])
                cf.LEDs_MODE_show[1] = np.flip(cf.LEDs_MODE_show[1])
                
                cf.LEDs_MODE = LEDs_MODE.copy()
                weight = np.array([5.5, 3.5, 2, 1, 0, 1, 2, 3.5, 5.5])
                cf.LEDs_weighted = np.multiply(cf.LEDs_MODE, weight)
                center_led = int((cf.LED_COLLUMS-1)/2)
                cf.CENTER_LEDs_MODE = cf.LEDs_MODE[:, center_led]
                cf.MAX_LEFT_LEDs =  np.max(cf.LEDs_weighted[:, :center_led], axis= 1)
                cf.MAX_RIGHT_LEDs =  np.max(cf.LEDs_weighted[:, center_led+1:], axis= 1)
                if time.time()-t_counter>0.03:
                    cf.Counter_Mode[:, 0:-1] =  cf.Counter_Mode[:, 1:].copy()
                    for i in range(4):
                        if np.sum(cf.LEDs_MODE[i])>0:
                            cf.Counter_Mode[i][-1] = 1
                        else:
                            cf.Counter_Mode[i][-1] = 0
                            
                        if np.sum(cf.Counter_Mode[i])>2 and cf.Counter_e[i] :
                            cf.Counter[i]+=1
                            cf.Counter_e[i] = False
                        if np.sum(cf.Counter_Mode[i])==0:
                            cf.Counter_e[i] = True
                    t_couter = time.time()
            for i in range(cf.CTHTs.shape[0]):
                cf.CTHTs[i] = GPIO.input(CTHTs[i])
        except Exception as e:
            cf.error_message +=1
            print (e)
    print('** read_inputs is stopped!')
    
##MPU sensor
from mpu import MPU9250
mpu9250 = MPU9250()
cf.t = time.time()
cf.gyro_z = 0
cf.angle = 0
cf.wz = 0.0
cf.last_gyro_z = 0.0

def update_mpu():
    print("** mpu started!")
    offset = np.zeros(200)
    for i in range(200):
        time.sleep(0.01)
        gyro = mpu9250.readGyro()
        offset[i] = gyro['z']
    offset_value = np.mean(offset[50:])
    print("** mpu cablirated!")
    while cf.RUN:
        time.sleep(0.01)
        try:
            gyro = mpu9250.readGyro()
            delta_t = time.time() -cf.t
            cf.t = time.time()
            cf.gyro_z = gyro['z'] -offset_value
            cf.wz += delta_t*(cf.gyro_z+cf.last_gyro_z)/2.0
            cf.angle = cf.wz
            cf.last_gyro_z =  cf.gyro_z
        except:
            pass
    print("** mpu stoped!")

def reset_mpu():
    cf.angle = 0
    cf.gyro_z = 0
    cf.wz = 0.0
    cf.last_gyro_z = 0.0
    
# Speed parameters
# MOVE DIRECTION
cf.UP = [1, 1, 1, 1]
cf.DOWN = [-1, -1, -1, -1]
cf.LEFT = [-1, 1, -1, 1]
cf.RIGHT = [1, -1, 1, -1]
cf.TURN_LEFT = [-1, 1, 1, -1]
cf.TURN_RIGHT = [1, -1, -1, 1]

a = 2
cf.R_ROTATE = (a*np.array(cf.RIGHT)+np.array(cf.TURN_LEFT))/(a+1)
cf.L_ROTATE = (a*np.array(cf.LEFT)+np.array(cf.TURN_RIGHT))/(a+1)

cf.PAUSE = [0, 0, 0, 0]
cf.cab = np.array([1 , 1, 0.99, 0.96])
cf.DIRECTIONs = [cf.UP, cf.LEFT, cf.DOWN, cf.RIGHT, cf.TURN_LEFT, cf.TURN_RIGHT, cf.R_ROTATE, cf.L_ROTATE, cf.PAUSE]
cf.NAMES_OF_DIR = ["UP", "LEFT", "DOWN", "RIGHT", "TURN_LEFT", "TURN_RIGHT", "R_ROTATE", "L_ROTATE"]
cf.last_message = ""



def set_servo(ser, angle):
    mess = "SER "+str(ser)+" "+str(angle)+" /"
    cf.uno.write(mess.encode())
    
def shot():
    print("Shoting....")
    mess = "SHOT 255 /"
    cf.uno.write(mess.encode())
    print("Shoted!")

def font_hand(direction):
    mess = "FHAND "+str(direction)+" /"
    cf.uno.write(mess.encode())
    time.sleep(1)
    while  cf.CTHTs[0] and cf.CTHTs[1]:
        time.sleep(0.02)
    mess = "FHAND  0 /"
    cf.uno.write(mess.encode())
    
def move(direction, speed):
    if cf.pause:
        this_speed = 0
    else:
        this_speed = speed
    direction = np.multiply(cf.cab, direction)
    message = "MOV"
    for i in direction:
        message+= " " + str(int(i*this_speed))
    message+= " /"
    try:
        cf.uno.write(message.encode())
    except:
        pass
    

def move_controller():
    print('** move_controllers is running...')
    while cf.RUN:
        time.sleep(0.02)
        move(cf.direction, cf.speed)
    print('** move_controllers is stopped!')

    
def show_leds_status(img):
    ## LEDS
    dis = 20
    for i in range(cf.LED_COLLUMS):
        begin = int((WIDTH - ((cf.LED_COLLUMS-1)*dis))/2)
        cv2.circle(img, ( begin+ dis*i, dis), 7, (255, 255, 255), -1)
        if cf.LEDs_MODE_show[0, i]:
            cv2.circle(img, ( begin+ dis*i, dis), 5, (0, 255, 0), -1)
    for i in range(cf.LED_COLLUMS):
        begin = int((HEIGHT - ((cf.LED_COLLUMS-1)*dis))/2)
        cv2.circle(img, (dis,  begin+ dis*i), 7, (255, 255, 255), -1)
        if cf.LEDs_MODE_show[1, i]:
            cv2.circle(img, (dis,  begin+ dis*i), 5, (0, 255, 0), -1)
    for i in range(cf.LED_COLLUMS):
        begin = int((HEIGHT - ((cf.LED_COLLUMS-1)*dis))/2)
        cv2.circle(img, (WIDTH - dis,  begin+ dis*i), 7, (255, 255, 255), -1)
        if cf.LEDs_MODE_show[3, i]:
            cv2.circle(img, (WIDTH - dis,  begin+ dis*i), 5, (0, 255, 0), -1)
    
    for i in range(cf.LED_COLLUMS):
        begin = int((WIDTH - ((cf.LED_COLLUMS-1)*dis))/2)
        cv2.circle(img, ( begin+ dis*i, HEIGHT - dis), 7, (255, 255, 255), -1)
        if cf.LEDs_MODE_show[2, i]:
            cv2.circle(img, ( begin+ dis*i, HEIGHT - dis), 5, (0, 255, 0), -1)
            
    for i in range(cf.CTHTs.shape[0]):
        begin = int((WIDTH - ((cf.CTHTs.shape[0]-1)*dis))/2)
        cv2.circle(img, ( begin+ dis*i, HEIGHT - dis-50), 7, (255, 255, 255), -1)
        if not cf.CTHTs[i]:
            cv2.circle(img, ( begin+ dis*i, HEIGHT - dis-50), 5, (0, 255, 0), -1)
            
    for i in range(3):
        begin = int((WIDTH - (2*dis))/2)
        cv2.circle(img, ( begin+ dis*i, dis+50), 7, (255, 255, 255), -1)
        if cf.Counter_Mode[0][i]==1:
            cv2.circle(img, ( begin+ dis*i, dis+50), 5, (0, 255, 0), -1)
    
    
    for i in range(4):
        cv2.putText(img, str(int(cf.Counter[i])), (120 + i*50, dis+100), font, 1, (255, 255, 255), 1, cv2.LINE_AA)
        #cv2.putText(img, str(round(cf.direction[i],2)), (50+ i*80, 200), font, 1, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(img,"speed: "+ str(round(cf.speed, 2)), (120, 180), font, 1, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(img, cf.file[8:], (120, 240), font, 1, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(img,"angle: "+ str(round(cf.angle, 2)), (120, 300), font, 1, (255, 255, 255), 1, cv2.LINE_AA)
    
    
def go_with_line(values):
    # PID parameters
    P = 0.3
    I = 0.0
    D = 0.1
    error_arr = np.zeros(5)
    ratio = 0
    
    # Other parameters
    UP_ratio = 5
    direction = cf.NAMES_OF_DIR.index(values[0])
    condition = values[1]
    time_start = time.time()
    cf.Counter = np.zeros(4, np.int32)
    reset_mpu()
    cf.pause = False
    print('=>> go_with_line:', values)
    DIRECTION = cf.DIRECTIONs[direction]
    rotate = False
    done = False
    time_point = time.time()-1
    check_counter =False
    while not cf.pause and cf.RUN and not done:
        if condition == "TIME":
            if time.time() - time_start>values[2]:
                done = True
        elif condition == "ANGLE":
            if abs(cf.angle - values[2])<5:
                done = True
        elif condition == "CTHT":
            if (not cf.CTHTs[2] or not cf.CTHTs[3]) and direction==0:
                done = True
            if not cf.CTHTs[4] and direction==2:
                done = True
        else:
            couter = cf.NAMES_OF_DIR.index(values[1])
            if cf.Counter[couter]== values[2]:
                done = True
            if cf.Counter[couter]== values[2]-1 :
                if direction<4 and  np.sum(cf.LEDs_MODE[direction])>4:
                    cf.speed = cf.low_speed
                
        total_weight = np.sum(cf.LEDs_MODE[direction])
        check_led = cf.CENTER_LEDs_MODE[direction]
        left_max = cf.MAX_LEFT_LEDs[direction]
        right_max = cf.MAX_RIGHT_LEDs[direction]
        error = left_max - right_max
        if total_weight == 0:
            error = error_arr[-1]
        delta_t = time.time() - time_point
        if error != error_arr[-1]:
            time_point = time.time()
        P_value = error*P
        I_value = np.sum(error_arr)*I
        D_value = D*(error -error_arr[-1])/delta_t
        ratio = P_value + I_value + D_value
        #print(error,P_value, I_value, D_value, ratio)
        error_arr[0:-1] = error_arr[1:].copy()
        error_arr[-1] = error
        vector = (UP_ratio*np.array(DIRECTION)+ratio*np.array(cf.TURN_LEFT))/(UP_ratio+abs(ratio))
        #print(vector)
        cf.direction = vector
        time.sleep(0.01)
    cf.speed = cf.default_speed
    cf.direction = cf.PAUSE

def go(values):
    direction = cf.NAMES_OF_DIR.index(values[0])
    condition = values[1]
    time_start = time.time()
    cf.Counter = np.zeros(4, np.int32)
    cf.pause = False
    print('=>> go:', values)
    reset_mpu()
    DIRECTION = cf.DIRECTIONs[direction]
    done = False
    cf.direction = DIRECTION
    while not cf.pause and cf.RUN and not done:
        if condition == "TIME":
            if time.time() - time_start>values[2]:
                done = True
        elif condition == "ANGLE":
            if abs(cf.angle - values[2])<5:
                done = True
        elif condition == "CTHT":
            if (not cf.CTHTs[2] or not cf.CTHTs[3]) and direction==0:
                done = True
            if not cf.CTHTs[4] and direction==2:
                done = True
        else:
            couter = cf.NAMES_OF_DIR.index(values[1])
            if cf.Counter[couter]== values[2]:
                done = True
            if cf.Counter[couter]== values[2]-1 :
                if direction<4 and  np.sum(cf.LEDs_MODE[direction])>4:
                    cf.speed = cf.low_speed
        time.sleep(0.01)
        
    
    cf.direction = cf.PAUSE
    cf.speed = cf.default_speed
    
def adjust():
    cf.speed = cf.low_speed
    cf.pause = False
    checking = False
    while cf.RUN and not cf.pause:
        left_max = cf.MAX_LEFT_LEDs[0]
        right_max = cf.MAX_RIGHT_LEDs[0]
        minus1 = left_max - right_max
        left_max = cf.MAX_LEFT_LEDs[2]
        right_max = cf.MAX_RIGHT_LEDs[2]
        minus2 = -left_max + right_max
        #print(minus1, minus2)
        if abs(minus1)<=1 and abs(minus2)<=1:
            if checking:
                print("OK")
                break
            cf.speed = cf.default_speed
            cf.direction = cf.PAUSE
            time.sleep(0.2)
            checking = True
        else:
            checking = False
        minus1 = np.sign(minus1)
        minus2 = np.sign(minus2)
        if minus1*minus2>0:
            cf.direction = minus1*np.array(cf.LEFT)
        if minus1*minus2<0:
            cf.direction = minus1*np.array(cf.TURN_LEFT)
        if minus1*minus2==0:
            cf.direction = np.sign(minus1-minus2)*np.array(cf.TURN_LEFT)
        time.sleep(0.1)
    
cf.command = ("nocommand", -1)

def set_command(command, value):
    cf.command = (command, value)
    cf.direction = [0, 0, 0, 0]
    cf.pause = True

def DecodeCommand():
    print("** DecodeCommand threading started!")
    while cf.RUN:
        time.sleep(0.1)      
        if cf.command[0] == 'go_with_line':
            set_command("None", -1)
            go_with_line(cf.command[1])
            
        if cf.command[0] == 'run_with_router':
            set_command("None", -1)
            run_with_router()
    print("** DecodeCommand threading stopped!")

trig = 12
echo = 16
GPIO.setup(trig, GPIO.OUT)
GPIO.setup(echo, GPIO.IN)

def distance():
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)
    tik = time.time()
    tok = time.time()
    
    while GPIO.input(echo)==0:
        tik = time.time()
    while GPIO.input(echo)==1:
        tok = time.time()
        
    distance = (tok-tik)*34300/2
    return distance

def check_ball(threshold):
    dis = distance()
    cf.ball = (dis<threshold)
    print("distance:", dis, cf.ball)

def pause():
    cf.command = ["nocommand", -1]
    cf.pause = True
    cf.direction = [0, 0, 0, 0]
    time.sleep(0.1)
    print("pausing")

def quit():
    cf.command = ["nocommand", -1]
    cf.pause = True
    cf.RUN = False
    cf.direction = [0, 0, 0, 0]
    time.sleep(0.1)
    print("quit!")
    
def run_with_router():
    cf.speed = cf.default_speed
    cf.pause =False
    direct_command = False
    print("=>Run with router...")
    stt = 0
    rt = open(cf.file, 'r')
    lines = rt.readlines()
    while cf.RUN and not cf.pause and stt<len(lines):
        if lines[stt][0]=="*":
            direct_command = not direct_command
            stt+=1
        while lines[stt][0]=="#":
            print(lines[stt][:-1])
            stt+=1
        if lines[stt][0]=="B":
            break
        if direct_command:
            eval(lines[stt])
            stt+=1
        else:
            step = eval(lines[stt])
            if len(step)==3:
                cf.speed = step[2]
            stt+=1
            time.sleep(0.1)
            if step[0] == "PASS" and not cf.ball:
                stt += step[1]
            if step[0] == "GO":
                go(step[1])
                cf.speed = cf.default_speed
            if step[0] == "LINE":
                go_with_line(step[1])
            if step[0] == "DELAY":
                time.sleep(step[1])
            if step[0] == "ADJ":
                print("=>> ADJ")
                adjust()
            if step[0] == "SER":
                set_servo(step[1][0], step[1][1])
            if step[0] == "SHOT":
                shot()
            if step[0] == "FHAND":
                font_hand(step[1])
            if len(step)==3:
                cf.speed = cf.default_speed
                
cf.RUN = True
cf.pause = False

cf.last_message = ""
cf.speed =255
cf.default_speed = cf.speed
cf.low_speed = 75
cf.medium_speed = 120
cf.direction = [0, 0, 0, 0]
cf.ratio = 0
cf.rotate = False
cf.colors = ['do', 'xanh']
cf.color = 1
cf.mode = 0

def set_6_led():
    mode = 4*cf.color+cf.mode
    cf.file ='/home/pi/Desktop/Git/Robocon2019/routers/'+cf.colors[cf.color]+str(cf.mode)+'.txt'
    for i in range(3):
        if mode%2==0:
            GPIO.output(cf.led6[i], False)
        else:
            GPIO.output(cf.led6[i], True)
        mode = int(mode/2)
    
    
set_6_led()
read_inputs_threading = threading.Thread(target=read_inputs)
read_inputs_threading.start()
time.sleep(0.1)
update_mpu_thread = threading.Thread(name = "update", target = update_mpu)
update_mpu_thread.start()
time.sleep(0.1)
move_controller_threading = threading.Thread(target=move_controller)
move_controller_threading.start()
time.sleep(0.1)
DecodeCommand_threading = threading.Thread(target=DecodeCommand)
DecodeCommand_threading.start()
time.sleep(0.1)

set_servo(0, 180)
time.sleep(0.05)
set_servo(1, 180)
time.sleep(0.05)
set_servo(2, 0)
time.sleep(0.05)
set_servo(3, 110)

while cf.RUN:
    img = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
    show_leds_status(img)
    cv2.imshow("robocon", img)
    k = cv2.waitKey(1)
    if k == ord('q'):
        quit()
        break
    if k == 32:
        reset_mpu()
        pause()
    if k ==ord('e'):
        adjust()
    if k== 184:
        cf.direction = cf.UP
        cf.pause = False
    if k== 178:
        cf.direction = cf.DOWN
        cf.pause = False
    if k== 180:
        cf.direction = cf.LEFT
        cf.pause = False
    if k== 182:
        cf.direction = cf.RIGHT
        cf.pause = False
    if k== ord('o'):
        cf.direction = cf.TURN_LEFT
        cf.pause = False
    if k== ord('p'):
        cf.direction = cf.TURN_RIGHT
        cf.pause = False
    if k== ord('r'):
        cf.pau0se = False
    if k == ord('z'):
        cf.LEDs_min = cf.LEDs.copy()
        print("min:") 
        print(cf.LEDs_min)
    if k == ord('x'):
        cf.LEDs_max[0] = cf.LEDs[0].copy()
        print("max UP:")
        print( cf.LEDs_max[0])
    if k == ord('c'):
        cf.LEDs_max[1] = cf.LEDs[1].copy()
        print("max LEFT:")
        print( cf.LEDs_max[1])
    if k == ord('v'):
        cf.LEDs_max[2] = cf.LEDs[2].copy()
        print("max RIGHT:")
        print( cf.LEDs_max[2])
    if k == ord('b'):
        cf.LEDs_max[3] = cf.LEDs[3].copy()
        print("max BOTTOM:")
        print( cf.LEDs_max[3])
    if k== ord('i'):
        print(cf.total_message, cf.error_message, 1/cf.total_message*cf.error_message,cf.error_probality)
        ping_to_arduino()  
    if k == ord('r'):
        set_command('run_with_router', [0])
    if k == ord ('n'):
        cf.LEDs_threshold = (cf.LEDs_min +cf.LEDs_max)/2
        np.save("LEDs_threshold.npy", cf.LEDs_threshold)
        print("Hieu:")
        print( -cf.LEDs_min +cf.LEDs_max)
        print("Threshold:")
        print( cf.LEDs_threshold)
    if k== ord('k'):
        font_hand(100)
    if k== ord('l'):
        font_hand(-100)
    if k== ord('j'):
        check_ball(15)
    if k == ord('w'):
        cf.mode = (cf.mode+1)%4
        set_6_led()
    if k == ord('s'):
        cf.color = (cf.color+1)%2
        set_6_led()
        
cv2.destroyAllWindows()
time.sleep(0.5)