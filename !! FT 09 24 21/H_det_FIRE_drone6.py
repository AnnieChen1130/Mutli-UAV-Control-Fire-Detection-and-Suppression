# coding: utf-8

# from pynq.pl import PL
# import pynq
import serial
# import RPi.GPIO as GPIO
import cv2
import numpy as np
import time
import dronekit as dk
from pymavlink import mavutil
from datetime import datetime
import os
import math


def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = dk.VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    time.sleep(8)
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


def send_ned_velocity(velocity_x, velocity_y, velocity_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    vehicle.send_mavlink(msg)
    vehicle.flush()


def goto_position_target_local_ned(north, east, down):  # THIS FUNCTION IS NOT NEEDED IN THIS SCRIPT
    # IT IS HERE FOR REFERENCE
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
        0b0000111111111000,  # type_mask (only positions enabled)
        north, east, down,  # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0,  # x, y, z velocity in m/s  (not used)
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()


# ----------------------------------------------KEEP IT THERE FOR REFERENCE
# connection_string = '/dev/ttyACM0'	#Establishing Connection With Flight Controller
# vehicle = dk.connect(connection_string, wait_ready=True, baud=115200)
# cmds = vehicle.commands
# cmds.download()
# cmds.wait_ready()
# waypoint1 = dk.LocationGlobalRelative(cmds[0].x, cmds[0].y, 3)  # Destination point 1
# ----------------------------------------------

# END of definitions!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# In[3]:

# In[4]:
# camera (input) configuration
# frame_in_w = 640
# frame_in_h = 480

# do need to set becasue for some reason the analog to digital convertor
# randomily set the size base on the computer and changing it will casue to not
# capture

# In[5]:
now = datetime.now()
time_stamp = now.strftime("%Y-%m-%d_%H_%M_%S")
with open(time_stamp + '_logs.txt', 'a+') as f:
    f.write('FLIGHT TEST: ' + time_stamp)
    f.write('\n')
    f.write('Settings threshold: Lower 45 Upper 50')
    f.write('\n')

cap = cv2.VideoCapture(0)
# videoIn.set(cv2.CAP_PROP_FRAME_WIDTH, frame_in_w);
# videoIn.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_in_h);
print("capture device is open: " + str(cap.isOpened()))
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'XVID'))
writer = cv2.VideoWriter(time_stamp + '.avi', cv2.VideoWriter_fourcc(*'XVID'), cap.get(cv2.CAP_PROP_FPS), (int(cap.get(3)), int(cap.get(4))))
#add another writer later for just fire images
#change the 20 to what the cap_prop_fps is set at
fps = cap.get(cv2.CAP_PROP_FPS)
w1 = cap.get(3)
h1 = cap.get(4)
centerx = w1/2
centery = h1/2
roistartw = cap.get(3) / 10
roistarth = cap.get(4) / 10

lower_red = np.array([0, 100, 100]) #change this to increase the red detection should work for under 15 meters
upper_red = np.array([0, 255, 255])
with open(time_stamp + '_logs.txt', 'a+') as f:
    f.write('Lower Red: ' + str(lower_red) + ' Upper Red: ' + str(upper_red))
    f.write('\n')
    f.write('FPS: ' + str(fps))
    f.write('\n')

# In[6]:
# initialize the HOG descriptor/person detector
# hog = cv2.HOGDescriptor()
# hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
# globalimage=np.zeros((640,480,3), np.uint8)
# not need to for fire

# Setting up GPIO
# GPIO.setmode(GPIO.BCM)
# GPIO.setup(18, GPIO.OUT)
# p = GPIO.PWM(18, 50)
# p.start(2.5)
# time.sleep(1)
# ...

# setting up xbee communication
# GPIO.setwarnings(False)
ser = serial.Serial(

    port='/dev/ttyUSB0',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)
# INITIALIZING DRONE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
connection_string = '/dev/ttyACM0'  # Establishing Connection With PIXHAWK
vehicle = dk.connect(connection_string, wait_ready=True, baud=115200)  # PIXHAWK is PLUGGED to NUC (RPi too?) VIA USB
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()

altitude = 8
speed = 1.0
waypoint1 = dk.LocationGlobalRelative(cmds[0].x, cmds[0].y, altitude)
arm_and_takeoff(altitude)
vehicle.airspeed = speed  # set drone speed to be used with simple_goto
vehicle.simple_goto(waypoint1)  # trying to reach 1st waypoint
with open(time_stamp + '_logs.txt', 'a+') as f:
    f.write('Altitude: ' + str(altitude))
    f.write('\n')
    f.write('Airspeed: ' + str(speed))
    f.write('\n')
# time.sleep(30)
# ----------------------------------------------
fire_found = 0
fire_counter = 7
#change the counter to 15 can change it later
while not fire_found:
    # read next image
    ret, frame = cap.read()
    frame = cv2.medianBlur(frame, 5) #add a image smoother medianBlur at 50%
    writer.write(frame)

    if not ret:
        print('Error Camera not connected')
        with open(time_stamp + '_logs.txt', 'a+') as f:
            f.write('Camera Error : RET returned False')
            f.write('\n')

    # cv2.imshow('original', frame)
    # frame = imutils.resize(frame, width=320)
    # frame3 = cv2.rectangle(frame, (int(roistartw)*3,int(roistarth)*3), (int(w)-int(roistartw)*3,int(h)-int(roistarth)*3), (0,255,0), 2)
    # cv2.imshow('roibox', frame3)
    #frame = frame[int(roistarth) * 1:int(h) - int(roistarth) * 1, int(roistartw) * 1:int(w) - int(roistartw) * 1]
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(frame_hsv, lower_red, upper_red)
    res = cv2.bitwise_and(frame, frame, mask=mask)
    gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    #add a write to see how many number of gray pixels are at every frame
    #maybe add a over number of pixel here later
    if cv2.countNonZero(gray) != 0:
        fire_counter = fire_counter - 1
        if fire_counter == 0:
            now = datetime.now()
            time_stamp1 = now.strftime("%Y-%m-%d_%H_%M_%S")
            cv2.imwrite(time_stamp + '_original.jpg', frame)
            cv2.imwrite(time_stamp + '_res.jpg', res)
            cv2.imwrite(time_stamp + '_gray.jpg', gray)
            cv2.imwrite(time_stamp + '_mask.jpg', mask)
            fire_found = 1
            print("FIRE")
            with open(time_stamp + '_logs.txt', 'a+') as f:
                f.write('FIRE AT: ' + time_stamp1)
                f.write('\n')
                f.write('Number of Red pixels: ' + str(cv2.countNonZero(gray)))
                f.write('\n')
    if cv2.countNonZero(gray) == 0:
        fire_counter = 10
    # cv2.imshow('ROI', frame)
    # cv2.imshow('mask', mask)
    # cv2.imshow('res', res)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("Exiting...")
        break
    cv2.waitKey(int((1 / int(fps)) * 1000))
# STOP Flying --------------------------------
send_ned_velocity(0, 0, 0)  # stop the vehicle
# sleepNrecord(2)
time.sleep(3)  # for 3 seconds

# CENTERING -----------------------------------------------------------------------------------------------------------
pixel_len = 0.537142856 #cm and if object is drone is 8 meter altitutde
N = 0
E = 0
flag = 0
while (vehicle.armed and flag != 15):
    ret, frame = cap.read()
    writer.write(frame)

    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(frame_hsv, lower_red, upper_red)
    res = cv2.bitwise_and(frame, frame, mask=mask)
    gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

    nlabels, labels, stats, centroids = cv2.connectedComponentsWithStats(gray, None, None, None, 8, cv2.CV_32S)
    contours, hierarchy = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if len(contours) == 1: #assuming there is only one contour
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            distance_w = w * pixel_len
            distance_h = h * pixel_len
            with open(time_stamp + '_logs.txt', 'a+') as f:
                f.write('Width: ' + str(distance_w))
                f.write('\n')
                f.write('Length:  ' + str(distance_h))
                f.write('\n')
            #print('w ' + str(distance_w) + ' ' + 'h ' + str(distance_h))
            #cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cx = round(centroids[1][0])
            cy = round(centroids[1][1])
            #cv2.line(frame, (round(centerx), round(centery)), (cx, cy), color=(255, 0, 0), thickness=5)
            distance = math.sqrt(((centerx - cx) ** 2) + ((centery - cy) ** 2))
            with open(time_stamp + '_logs.txt', 'a+') as f:
                f.write('Distance: ' + str(distance*pixel_len))
                f.write('\n')

            if (cy < centery and cx < centerx): #I
                #print("I")
                flag = flag + 1
                N = (abs(cy-centery) * pixel_len)/100
                E = -(abs(cx-centerx) * pixel_len)/100
                with open(time_stamp + '_logs.txt', 'a+') as f:
                    f.write('N: ' + str(N))
                    f.write('\n')
                    f.write('E:  ' + str(E))
                    f.write('\n')
            elif (cy < centery and cx >= centerx): #II
                #print("II")
                flag = flag + 1
                N = (abs(cy-centery) * pixel_len)/100
                E = (abs(cx-centerx) * pixel_len)/100
                with open(time_stamp + '_logs.txt', 'a+') as f:
                    f.write('N: ' + str(N))
                    f.write('\n')
                    f.write('E:  ' + str(E))
                    f.write('\n')
            elif (cy >= centery and cx < centerx): #III
                #print("III")
                flag = flag + 1
                N = -(abs(cy-centery) * pixel_len)/100
                E = -(abs(cx-centerx) * pixel_len)/100
                with open(time_stamp + '_logs.txt', 'a+') as f:
                    f.write('N: ' + str(N))
                    f.write('\n')
                    f.write('E:  ' + str(E))
                    f.write('\n')
            elif (cy >= centery and cx >= centerx): #IV
                #print("IV")
                flag = flag + 1
                N = -(abs(cy-centery) * pixel_len)/100
                E = (abs(cx-centerx) * pixel_len)/100
                with open(time_stamp + '_logs.txt', 'a+') as f:
                    f.write('N: ' + str(N))
                    f.write('\n')
                    f.write('E:  ' + str(E))
                    f.write('\n')
    cv2.waitKey(int((1 / int(fps)) * 1000))
# CENTERING -----------------------------------------------------------------------------------------------------------

#Move to new location -------------------------------------------------------------------------------------------------

goto_position_target_local_ned(N, E, 0)

send_ned_velocity(0, 0, 0)  # stop the vehicle
# sleepNrecord(2)
time.sleep(3)

#Move to new location -------------------------------------------------------------------------------------------------


# READ CURRENT COORDINATES FROM PIXHAWK-------------------
lat = vehicle.location.global_relative_frame.lat  # get the current latitude
lon = vehicle.location.global_relative_frame.lon  # get the current longitude
coords = "RESCUE " + str(lat) + " " + str(lon)
with open(time_stamp + '_logs.txt', 'a+') as f:
    f.write('Coordinates: ' + coords)
    f.write('\n')
# TRANSMIT CURRENT COORDINATES TO RESCUE DR --------------
ser.write(coords.encode())

# RETURN HOME CODE ----------------------------
vehicle.mode = dk.VehicleMode("RTL")
# ANOTHER LOOP
while vehicle.armed:
    ret, frame = cap.read()
    writer.write(frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("Exiting...")
        break
    cv2.waitKey(int((1 / int(fps)) * 1000))
# time.sleep(20)

# ---------------------------------------------
vehicle.mode = dk.VehicleMode("LAND")
vehicle.flush()


# add to keep recourding??
# or move code to into look and if statement
# and add another flag
