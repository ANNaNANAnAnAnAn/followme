import time 
import serial 
import numpy as np 

DIST_SENSORS = 94 
log_file = open("output.txt", "w")


def log_print(*args, **kwargs):
    print(*args, **kwargs)                     
    print(*args, **kwargs, file=log_file)      
    log_file.flush()

def openPort(port, baud):
    ser = serial.Serial(
        port=port,
        baudrate=baud,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        rtscts=True,
        timeout=10)
    return ser      

def calibr(ser):
    i=0
    while i < 1:
        line = ser.readline()
        if len(line) >60:
            line = str(line)
            line = line.split(',')
            line = line[2]
            i+=1
    
    return int(line)

def calculateDist(sensorleft, sensorright):
    angle_left = calibr(sensorleft)
    angle_right = calibr(sensorright)
    angle_left_rad = np.radians(abs(angle_left))
    angle_right_rad = np.radians(abs(angle_right))
    half_dist = DIST_SENSORS / 2.0

    side_l = DIST_SENSORS *  np.sin(angle_right_rad)/np.sin(angle_left_rad)

    distance = np.sqrt(np.square(side_l) + np.square(half_dist)+ 2 * side_l* half_dist * np.cos(angle_left_rad))
    return distance

###############################################################################
# MAIN
###############################################################################
aoaBoardLeft = openPort('/dev/ttyUSB3', 115200)
aoaBoardRight = openPort('/dev/ttyUSB2', 1000000)

while True: 
    distance = calculateDist(aoaBoardLeft, aoaBoardRight)
    log_print(f"Distance: {distance:.2f} cm")
    
    if distance < 500 and distance > 300:
        log_print("Object detected, moving forward")
    elif distance < 300: 
        log_print("Object too close, stopping")

    time.sleep(0.1) 

