#!/usr/bin/python           # This is server.py file

############################################################################
# Test
#  mosquitto_pub -t "tablet/robot_cmd" -m "0 1 0
#  mosquitto_sub -t "tablet/robotstatus" -q 1
#  mosquitto_pub -t "tablet/heartbeat" -m "122 192.168.8.181 1 5000 "
############################################################################
# tst2
#import paho.mqtt.client as MQTT
import math
import queue
import time
import socket               # Import socket module
import _thread
import subprocess
import os
import serial
import numpy as np


# Global variables
#mqtt_client = MQTT.Client();

###############################################################################
# MQTT
###############################################################################
# def onConnect(client, userdata, flags, rc):
#     print("TABLET Connected with result code " + str(rc));
#     mqtt_client.subscribe("ugv/followme")
#     mqtt_client.message_callback_add("ugv/followme", onFollowMe)

# def onFollowMe(client, userdata, message):
#     msg = message.payload.decode('UTF-8');
#     #print("onFollowMe: " + msg)

# def heartbeat(ang, dist):
#     msg = str(ang) + " " + str(dist) + " "
#     #print("HB: " + msg)
#     mqtt_client.publish("ugv/followme", msg);

def openPort(port,baud):
    ser = serial.Serial(
        port=port,
        baudrate=baud,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        rtscts= True,
        timeout=10)
    return ser

# def calc_distance(aleft, aright, dst): 
#     halfdist = dst / 2

#     ac1 = math.radians(90 - abs(aleft))
#     ac2 = math.radians(90 - abs(aright))
#     at1 = math.tan(ac1)
#     at2 = math.tan(ac2)

#     fdist1 = halfdist * at1 
#     fdist2 = halfdist * at2
#     fdist = (fdist1 + fdist2) / 2.0
#     return fdist

    
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

def vid(nolases):
    sumx,sumy = 0,0
    for i in range(10):
        x = nolases[i][0]
        y = nolases[i][1]
        sumx += x
        sumy += y
    vidx = sumx/10
    vidy = sumy/10
    return vidx,vidy

def getInfo(sensor1):
    dati1 = calibr(sensor1)
    print(str(dati1))
 #   dati2 = calibr(sensor2)

    angleSensLeft = dati1
  #  angleSensRight = dati2

    # angleSensLeftA = 90 - angleSensLeft

    #angleSensRightA = 90 + angleSensRight
    #oppositeAngleA = 180 - (angleSensRight + angleSensLeft)

    #if (oppositeAngle >= 180) or (oppositeAngle <= 0):

    #    return -1.0, -1.0

    #right_r = math.sin(math.radians(oppositeAngleA)) * math.sin(math.radians(angleSensLeftA))
    #right_l = math.sin(math.radians(oppositeAngleA)) * math.sin(math.radians(angleSensRightA))

    #sx = (angleSensLeft + angleSensRight) / 2

    #m_c = calc_distance(angleSensLeft,angleSensRight,DIST_BETWEEN_SENSORS);
    #print("angle: " + str(angleSensLeft) + " " + str(angleSensRight) + " " + str(sx) + " : " + str(right_l) + " " + str(right_r) )

    #if ((right_r != 0.0) and (right_l != 0.0)): 
        #rightSideLength = DIST_BETWEEN_SENSORS / right_r
        #leftSideLenght = DIST_BETWEEN_SENSORS / right_l
        #m_c = (1/2)* math.sqrt(2 * rightSideLength * rightSideLength + 2 * leftSideLenght * leftSideLenght - DIST_BETWEEN_SENSORS * DIST_BETWEEN_SENSORS)
        
        #leftSideLenght = DIST_BETWEEN_SENSORS / math.sin(math.radians(oppositeAngleA)) * math.sin(math.radians(angleSensRightA))
        #m_c = (1/2) * math.sqrt(2 * rightSideLength * rightSideLength + 2 * leftSideLenght * leftSideLenght - DIST_BETWEEN_SENSORS * DIST_BETWEEN_SENSORS)

        #rightSideLength = DIST_BETWEEN_SENSORS / math.sin(math.radians(oppositeAngleA)) * math.sin(math.radians(angleSensLeftA))

        #m_c = math.sqrt(rightSideLength * rightSideLength + DIST_BETWEEN_SENSORS/2 * DIST_BETWEEN_SENSORS/2 - 2 * rightSideLength * DIST_BETWEEN_SENSORS/2 * math.cos(math.radians(angleSensRightA)))
        #m_c = 1.0 / ((DIST_BETWEEN_SENSORS / 2.0) * math.tan(math.radians(angleSensRight)))
        #m_c = 1.0
    #else:
        #m_c = 0.0
    print("Angle:  [" + str(angleSensLeft)+","+ "] : Distabce: none " )
    return angleSensLeft

###############################################################################
# MAIN
###############################################################################
# Connect to MQTT broker
#mqtt_client = MQTT.Client();
# mqtt_client.on_connect = onConnect;

# hostip = "127.0.0.1"
# #hostip = "212.3.210.249"

# mqtt_client.connect_async(host=hostip, port=1883, keepalive=60);
# #mqtt_client.on_message = on_message;s
# mqtt_client.loop_start();

# print('TABLET service started!')


# cmd_queue = queue.Queue()
# # Queue.get(block=True, timeout=None)
# #hbcounter,localIP,_stini.tbl_gsttype, _stini.tbl_gstport, istop, ilight, idirection


# #gst_exec = "gst-launch-1.0 udpsrc port=5000 ! application/x-rtp, encoding-name=JPEG,payload=26 ! rtpjpegdepay ! jpegdec ! autovideosink"
# #  gst-launch-1.0 udpsrc port=5000 ! application/x-rtp ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink
# time.sleep(1.0)

sensor2 = openPort("/dev/ttyUSB0",1000000)

c = 600 #in mm

hb_lasttm = time.time()
tmdelta = 0.2 # how long think that tablet is alive (secs)
while(True):
    curtm = time.time()
    deltaatm = curtm - hb_lasttm

    angle = getInfo(sensor2)
    if (deltaatm > tmdelta):
 #           heartbeat(angle, distance)
            hb_lasttm = curtm
            print(angle)
    time.sleep(1)
    #sendSettings('Test',47)

sensor1.close()
sensor2.close()
