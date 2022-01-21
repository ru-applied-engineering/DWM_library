#!/usr/bin/python3
import decawave_ble as ble
import pandas as pd
import numpy as np



#This script was made to evaluate the DWM1001 Dev Boards signal strength by putting them on metal coated windows
#To see if the signal can get through the windows.
#Read the paper of the independent course, then you will understand how this was setup.

#TODO: Insert Independent Study PDF in the repository.




nr=1
filename = "Indoor" + str(nr)  #Indoor, Outdoor


#Getting all connected devices in the network. Do not work with the Navio Raspberry Pi Image(EMLID)!!!

RSSI_data = []
dev_names = []
Data = {}
#Names of the DWM1001 devices in the setup network in the DRTLS android app, change the names if needed.
names = ["DWTAG1","DWX1X1","DWX2X2","DWX3X3","DWY1Y1","DWY2Y2","DWY3Y3"]


N = 25   #First run, test it with a small N value.
for i in range(0,N):
   decawave_scan_entry = ble.get_decawave_scan_entries()
   print('Scanning....')
   devs= {}
   for dse in decawave_scan_entry:
       dev = ble.DecawaveDevice(dse)
       scan_data = dev.scan_data()
       print(scan_data['device_name'], scan_data['rssi'])
       devs[scan_data['device_name']] = scan_data['rssi']

   keys = [*devs]
   print(keys)
   for n in names:
       if n not in keys:
          devs[n] = -100
   print(devs)
   RSSI_data.append(devs)   

df = pd.DataFrame(data=RSSI_data,columns = names)
print(df)
df.to_csv(filename + '.csv',index=False,sep=',') #Next person, run with a small N value, to check if format in csv file is as expected.













