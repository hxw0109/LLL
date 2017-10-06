import math
import os
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

frmCounter = 0
lineCounter = 0

resultFilename = "C:/Users/hux/Desktop/ML/Lidar/Data/test"

plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d') #facecolor='r'



with open(resultFilename, 'r') as f:

	line = f.readline()
	lineCounter+=1

	while line:
		if lineCounter%8 == 1:
			# Plotting
			
			# plt.pause(0.1)
			plt.waitforbuttonpress()
			plt.cla()


			frameid = int(line)
			timestamp = float(f.readline())
			lineCounter+=1
			print([frameid,timestamp])

			xp = [float(num) if num else [] for num in f.readline().rstrip(']\n').lstrip('[').split(',')]
			yp = [float(num) if num else [] for num in f.readline().rstrip(']\n').lstrip('[').split(',')]
			zp = [float(num) if num else [] for num in f.readline().rstrip(']\n').lstrip('[').split(',')]
			xr = [float(num) if num else [] for num in f.readline().rstrip(']\n').lstrip('[').split(',')]
			yr = [float(num) if num else [] for num in f.readline().rstrip(']\n').lstrip('[').split(',')]
			zr = [float(num) if num else [] for num in f.readline().rstrip(']\n').lstrip('[').split(',')]

			
			ax.scatter(xr,yr,zr, c='r', marker='.', s=20)
			ax.scatter(xp,yp,zp, c='b', marker='.', s=10)
			ax.set_title('Frame: ' + str(frameid))
			ax.set_xlim3d([-30, 30])
			ax.set_xlabel('X')
			ax.set_ylim3d([0, 30])
			ax.set_ylabel('Y')
			ax.set_zlim3d([0, 8])
			ax.set_zlabel('Z')
			plt.draw()


			lineCounter+=6

		line = f.readline()
		lineCounter+=1
plt.show(block=True)