#!/usr/bin/python3
'''
   This file are the merged work of the position system script using the beacon network in Lab V207, Reykjavik University
   and inserting lat,lon and altitude to Mission Planner.

   This file works with the Navio autopilot and Raspberry Pi image for the Navio.
   As the Navio do not have Bluetooth, then communication need to be done by sending 
   commands via uart to DWM to get the information e.g tag position within the network.

   API Guide can be retrieved from this site: https://www.decawave.com/wp-content/uploads/2019/01/DWM1001-API-Guide-2.2.pdf
   If not, then search for the api guide for DWM1001.

   The scriipt uses pymavlink and some scripts, tutorials can be found: 
   https://www.ardusub.com/developers/pymavlink.html

'''
from geographiclib.geodesic import Geodesic
from pymavlink import mavutil
import math
import pandas as pd
import numpy as np
import time
import datetime
import signal
import sys
import serial
import struct
import re

###############################################################
#WSG84 as the Coordinate Reference System.
geod = Geodesic.WGS84
###############################################################

##############################################################
#Metrics: Anchor has mm metric which will be converted to meters
mm = 1000
##############################################################

###############################################################
#Latitude and Longitude, chosen and calculated from Google Maps.
START_LAT, START_LON = 64.123918,-21.925399
END_LAT,END_LON = 64.123909,-21.925390
INIT_LAT,INIT_LON = 64.12390919390242,-21.925490803281907
ANCH_LAT,ANCH_LON = 64.12384421369376,-21.925459771654022
################################################################


################################################################
#global file
class Killer:
   '''
      SIGNAL HANDLER CLASS,
      KILLS AS EXPECTED WITH CTRL+C.
   '''
   kill_now = False
   def __init__(self):
       signal.signal(signal.SIGINT,self.quit_)
       signal.signal(signal.SIGTERM,self.quit_)

   def quit_(self,*args):
       self.kill_now = True
       #global file
       #file.flush()
       #file.close()
###############################################################

###############################################################
class Decawave:
   def __init__(self, Port = '/dev/ttyACM0', debug=0):
        self.x = 0.0 #X pos
        self.y = 0.0 #Y pos
        self.z = 0.0 #Z pos
        self.q = 0.0 #Quality

        self.init_x = 0.0
        self.init_y = 0.0
        self.init_z = 0.0


        self.debug = debug
        self.connect_to_node(Port)


   def write_uart(self,cmd,tsleep):
        self.serialPort.write(cmd.encode())
        time.sleep(tsleep)

   def clear_uart(self):
       self.serialPort.reset_input_buffer()
       self.serialPort.reset_output_buffer()

   def write_debug(self,db_msg):
       if(self.debug):
         print(db_msg)

   def connect_to_node(self, Port):
       self.serialPort = serial.Serial(port=Port,baudrate = 115200, timeout=10)
       self.write_debug(f"Connected to {self.serialPort.name}")
       self.write_uart("\r\r",1)

   def disconnect_to_node(self):
       self.serialPort.close()
       self.write_debug(f"Device disconnected!")
   
   def get_pos():
       return self.x,self.y,self.z
  
   def xyz(self):
       try:
           self.clear_uart()
           for i in range(10):
               self.write_uart("apg\r",0.1)
               data = self.serialPort.readline().strip()
               if len(data) >= 15:
                  data = data[5:].split()
                  data = [d.split(b':')[1] for d in data]
                  if len(data) == 4:
                     self.x = int(data[0])
                     self.y = int(data[1])
                     self.z = int(data[2])
                     self.q = int(data[3])
                     #print(f"(x,y,z,q) = {self.x},{self.y},{self.z},{self.q}")
                     break

                  #if(i==10):
                     #print("Failed to get pos, 10 times") #Hopefully not needed, just to see if this 

           self.clear_uart()     
       except Exception as ex:
           print(ex)
########################################################################################################





########################################################################################################
#Function for calculating Bearing from true north.
def bearing_angle(start_lat,start_lon,end_lat,end_lon):
    #Function: Calculates the bearing angle from True North

    g = geod.Inverse(start_lat,start_lon,end_lat,end_lon) 
    d = g['s12'] #s12 is for distance, Check GeographicLib API
    TN = geod.Direct(start_lat,start_lon,0,d) # True North is at the 0 degree heading/bearing..
    
    #These calculations, come from a weblink, search for: Find Bearing Angle between two GPS lat/lon coordinates. Check Paper also!!
    X = math.cos(math.radians(end_lat))*math.sin(math.radians(end_lon) - math.radians(TN['lon2']))
    Y = math.cos(math.radians(TN['lat2']))*math.sin(math.radians(end_lat)) - math.sin(math.radians(TN['lat2']))*math.cos(math.radians(end_lat))*math.cos(math.radians(end_lon) - math.radians(TN['lon2']))
    B = (math.degrees(math.atan2(X,Y)) + 360) % 360 #The Bearing Angle Calculation
    return B

########################################################################################################


def main():
    
    #Pymavlink
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14555',source_component=1)
    boot_t = int(time.time())
    flag = None
    print("Connecting")
    flag = master.wait_heartbeat()
#    if flag is None:
#        return
    print("Connected!!")
    master.mav_type = 14
    print("Mav_type is set, has: ",master.mav_type)
    print("Target system: ", master.target_system)
    print("Target component: ", master.target_component)
    print("Source system: ", master.source_system)
    print("Source component: ",master.source_component)


    flags = (mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |\
         mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_VERT |\
         mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY)

    #Positon system, beacon.
    KILL = Killer()
    B = bearing_angle(START_LAT,START_LON,END_LAT,END_LON) 
    pos_avg = []
    GPS = False
    i=1
    DWM = Decawave(debug=1)
    #DWM.get_anchor_pos()
    #zi = DWM.init_z
    while not KILL.kill_now:
        try:
            DWM.xyz()
            x,y,z = DWM.x/mm, DWM.y/mm, DWM.z/mm
        except Exception as ex:
            print(ex)
        else:
            print(f"x:{x},y:{y}, z: {z}")
            pos_avg.append([x,y,z])
            if(len(pos_avg) >= 5):
               avg = np.array(pos_avg[-5:])
               x,y,z = np.mean(avg,axis=0)
               print(f"AVG: x:{x}, y:{y}, z:{z}")
               GPS = True
               if(len(pos_avg) >=1000):
                  temp = pos_avg[-5:]
                  pos_avg = []
                  for t in temp:
                      pos_avg.append(t)
            if(GPS):
                if(i % 5 == 1): #Get 2Hz output. 10Hz -> 5 GPS values, takes one of them , equals 2Hz.
                  d_tag = math.sqrt((x**2 + y**2))
                  th = (math.degrees(math.atan2(y,x)) + 360) % 360
                  tag_B = B + 90 - th
                  g = geod.Direct(INIT_LAT,INIT_LON,tag_B,d_tag)
                  #print(f"Tag lat: {g['lat2']}, lon: {g['lon2']}")
                  lat = int(g['lat2'] * 1e7)
                  lon = int(g['lon2'] * 1e7)
                  print(f"Tag lat: {lat}, lon: {lon}")
                  GPS = False
                  master.mav.gps_input_send(
                      int(1.0e6 *(time.time() - boot_t)),  # Timestamp (micros since boot or Unix epoch)
                      0,  # ID of the GPS for multiple GPS inputs
                      # Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).
                      # All other fields must be provided.
                      flags,
                      0,  # GPS time (milliseconds from start of GPS week)
                      0,  # GPS week number
                      3,  # 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
                      lat,  # Latitude (WGS84), in degrees * 1E7
                      lon,  # Longitude (WGS84), in degrees * 1E7
                      z,  # Altitude (AMSL, not WGS84), in m (positive for up)
                      0.7,  # GPS HDOP horizontal dilution of position in m
                      1,  # GPS VDOP vertical dilution of position in m
                      0,  # GPS velocity in m/s in NORTH direction in earth-fixed NED frame
                      0,  # GPS velocity in m/s in EAST direction in earth-fixed NED frame
                      0,  # GPS velocity in m/s in DOWN direction in earth-fixed NED frame
                      0,  # GPS speed accuracy in m/s
                      1,  # GPS horizontal accuracy in m
                      1,  # GPS vertical accuracy in m
                      0)   # Number of satellites visible.
                  print("GPS Sent to MP")

                i+=1
                if(i >= 1000):
                    i=1

            
            









if __name__ == '__main__':
    main()


