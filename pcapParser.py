from tkinter import Tk
from tkinter.filedialog import askopenfilename, asksaveasfilename


from collections import deque
import math
import os
import numpy as np


UDPportSearchArray = [b'\x09', b'\x40', b'\x09', b'\x40', b'\x04', b'\xbe', b'\x00', b'\x00']
LastItemInSearch = b'\x00'
# Tk().withdraw() # we don't want a full GUI, so keep the root window from appearing

# ftypes = [
#   ('Velodyne Native Pcap', '*.pcap')]
# ftypes2 = [
#   ('dump', '*')]

# filename = askopenfilename(filetypes=ftypes) # show an "Open" dialog box and return the path to the selected file
# print(filename)
# if filename =="":
# 	exit()

# recordingName = asksaveasfilename (filetypes=ftypes2)
# print(recordingName)
# if recordingName =="":
#   exit()

filename = "C:/Users/hux/Desktop/ML/Lidar/Data/test.pcap"
recordingName = "C:/Users/hux/Desktop/ML/Lidar/Data/test"

resultFile = open(recordingName ,"w")

#User Functions
def getXYZ(a,w,d):      #calculated XYZ from azimuth, elevation, and distance
    p = d * math.cos(math.radians(w))  #calculate 2d ground Projection
    z = d * math.sin(math.radians(w))
    x = p * math.sin(math.radians(a))
    y = p * math.cos(math.radians(a))
    return x,y,z



detectionQueue = deque(8*[0], 8) # queue to detect the data frame
filteredBeamRange = 8 # Count from top beam downwards
filteredAngle = [285,75]
blockSize = 12
msgCounter = 0
frmCounter = 0
timestamp_temp = 0
posVar = 50 # Variance filter number, 50mm=5cm
reflThre = 140 # 0-255

# Frame data initilization
timestamp_arr = []
azi_arr = []
cordi_arr = [] # Units of X Y Z are in mm
r_arr = []
flag_position = []
flag_reflectivity = []


with open(filename, "rb") as binaryData:
  byte = binaryData.read(1)
  # Do stuff with byte.
  while byte != b"": # or just while byte:

    byte = binaryData.read(1)
    detectionQueue.append(byte)

    # read a whole frame of 1206 byte of data once its found
    if byte == LastItemInSearch and list(detectionQueue) == UDPportSearchArray:

      msg = binaryData.read(1206)
      ## Msg structure
      ##  1     2   ...   12                 
      # xFFEE xFFEE ... xFFEE    --2 bytes
      # azim1 azim2 ... azim12   --2 bytes
      # D/R 0  ...  ...  ...     --3 bytes
      # D/R 1  ...  ...  ...     --3 bytes
      #  ...   ...  ...  ...     --3 bytes
      #  ...   ...  ...  ...     --3 bytes
      # D/R 15 ...  ...  ...     --3 bytes
      # D/R 0  ...  ...  ...     --3 bytes
      # D/R 1  ...  ...  ...     --3 bytes
      #  ...   ...  ...  ...     --3 bytes
      #  ...   ...  ...  ...     --3 bytes
      # D/R 15 ...  ...  ...     --3 bytes
      # +6 bytes of timestamp + factory setting at the end

      azimuth_first = ((msg[3] << 8)| msg[2])/100
      azimuth_last = ((msg[1103] << 8)| msg[1102])/100
      

      # The range in concern: 270 to 90
      if azimuth_first >= filteredAngle[0] or azimuth_last <= filteredAngle[1]:

        # Determine if the previous frame has completed, if so, do the determination calculation and reset the buffer for new frame
        azi_last_reference = 360.5 if not azi_arr else azi_arr[-1]
        # print(azi_arr)

        if (azimuth_first-azi_last_reference)>(filteredAngle[0]-filteredAngle[1]-10):
          frmCounter += 1
          print(frmCounter)
          timestamp_arr.append(timestamp_temp)
          print(timestamp_temp)
          # print(azi_arr)
          # Index of position/reflectivity to visualize
          positionIndex = [i for i,x in enumerate(flag_position) if x == 1]
          reflectivityIndex = [i for i,x in enumerate(flag_reflectivity) if x == 1]

          xp = [round(cordi_arr[index][0]/1000,2) for i,index in enumerate(positionIndex)]
          yp = [round(cordi_arr[index][1]/1000,2) for i,index in enumerate(positionIndex)]
          zp = [round(cordi_arr[index][2]/1000,2) for i,index in enumerate(positionIndex)]

          xr = [round(cordi_arr[index][0]/1000,2) for i,index in enumerate(reflectivityIndex)]
          yr = [round(cordi_arr[index][1]/1000,2) for i,index in enumerate(reflectivityIndex)]
          zr = [round(cordi_arr[index][2]/1000,2) for i,index in enumerate(reflectivityIndex)]

          resultFile.write(str(frmCounter)+"\n"+
            str(round(timestamp_temp,2))+"\n")
          resultFile.write(str(xp)+'\n'+
            str(yp)+'\n'+
            str(zp)+'\n'+
            str(xr)+'\n'+
            str(yr)+'\n'+
            str(zr)+'\n')
          

          # Reset frame buffer
          azi_arr = []
          cordi_arr = []
          r_arr = []
          flag_position = []
          flag_reflectivity = []

        # Get timestamp
        timestamp_temp = (msg[1203]<<24 | msg[1202]<<16 | msg[1201]<<8 | msg[1200])/1000000
        aziUp = []
        aziLo = []
        # Calculate azimuth
        for i in range(blockSize):
            aziUp.append(((msg[3+(i*100)] << 8)| msg[2+(i*100)])/100)
        for i in range(blockSize-1):
            delta = (aziUp[i+1]-aziUp[i]) if aziUp[i+1]>aziUp[i] else (aziUp[i+1]-aziUp[i]+360)
            aziLo_temp = aziUp[i]+(delta)/2
            aziLo_temp = aziLo_temp if aziLo_temp<360 else aziLo_temp-360
            aziLo.append(round(aziLo_temp,2))
        last_azimuth = aziUp[-1]+(delta)/2
        last_azimuth = last_azimuth if last_azimuth<360 else last_azimuth-360
        aziLo.append(round(last_azimuth,2))
        for i in range(blockSize):
          azi_arr.append(aziUp[i])
          azi_arr.append(aziLo[i])

        # Get details of cloud points
        for j in range(blockSize*2):
          [x_temp,y_temp,z_temp] = [100000,100000,100000]

          for i_beam in range(filteredBeamRange): # Count from top down
            altitude = 15-i_beam*2 # degree of the beam
            i = 15-i_beam*2 if i_beam<8 else 15-i_beam*2-1 # calculate beam altitude index (hint:-15,1,-13,3,...,-1,15)

            dist = ((msg[i*3+(5+48*(j%2))+(j//2*100)] << 8) | msg[i*3+(4+48*(j%2))+(j//2*100)])*2   # Distance multiply by 2mm
            r = msg[i*3+(6+48*(j%2))+(j//2*100)]

            if altitude <0:
              print("Alarm"+str(altitude))

            if j%2:
              [x,y,z] = getXYZ(aziLo[j//2], altitude, dist)
            else:
              [x,y,z] = getXYZ(aziUp[j//2], altitude, dist)
            cordi_arr.append([round(x,1),round(y,1),round(z,1)])

            r_arr.append(r)

            # Calculate flag
            if y != 0:
              if r>reflThre:
                flag_reflectivity.append(1)
              else:
                flag_reflectivity.append(0)
            else:
              flag_reflectivity.append(0)

            if x != 0: # and y != 0
              if abs(x-x_temp)<posVar and abs(y-y_temp)<posVar:
                flag_position[-1] = 1
                flag_position.append(1)
              else:
                flag_position.append(0)
            else:
              flag_position.append(0)

            [x_temp,y_temp,z_temp] = [x,y,z]
