import numpy as np
from collections import deque
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


np.set_printoptions(threshold=np.nan)

class Sign(object):

	def __init__(self,name,point_index:'tuple',distance,s):
		self.name = name
		self.counter = 1
		self.distance = distance
		self.points = [[point_index[0]],[point_index[1]]]
		self.lastPoint = point_index
		self.center = point_index
		self.size = [1,1]
		self.s = s

	def update(self,point_index,distance,s):
		self.points[0].append(point_index[0])
		self.points[1].append(point_index[1])
		self.distance = (self.distance*self.counter+distance)/(self.counter+1)
		self.counter += 1
		self.lastPoint = point_index
		if s > self.s:
			self.s = s
			self.center = point_index

def pointRetained(point_cloud,point:'tuple',filter_criteria:'list'):

	rectangle_center = point # row, col
	rectangle_offset = filter_criteria[2:4]
	criteria = filter_criteria[4]
	# col_offset = 
	row1 = rectangle_center[0]-rectangle_offset[0] if (rectangle_center[0]-rectangle_offset[0])>=0 else 0
	row2 = rectangle_center[0]+rectangle_offset[0]+1
	col1 = rectangle_center[1]-rectangle_offset[1] if (rectangle_center[1]-rectangle_offset[1])>=0 else 0
	col2 = rectangle_center[1]+rectangle_offset[1]+1

	rectangle = point_cloud[row1:row2:1,col1:col2:1]
	# print(rectangle,'\n')
	counter = sum(sum(rectangle))
	decision = True if counter >= criteria else False
	return decision, counter

def filterByReflectivity(reflectivity_array,distance_array,filter_criteria,grouping_rule):

	r_decision = np.zeros(reflectivity_array.shape)
	idx_tuple_r = np.where((reflectivity_array>=filter_criteria[0]))

	if idx_tuple_r:

		# Filter by distance if necessary
		# idx_d = np.where(distance_array[idx_tuple_r]<distance_threshold[0])
		# idx_r_row = idx_tuple_r[0][idx_d]
		# idx_r_col = idx_tuple_r[1][idx_d]
		# idx_r = [idx_r_row, idx_r_col]


		# Filter by pre defined size
		r_decision[idx_tuple_r] = 1

		i = 0
		num_sign = 1
		group_signs = []
		point_previous = (0,0)
		while i<len(idx_tuple_r[0]):

			point = (idx_tuple_r[0][i], idx_tuple_r[1][i])
			distance = distance_array[point]
			point_retained, s = pointRetained(r_decision,point,filter_criteria)
			if not point_retained or distance<=filter_criteria[1]:
				r_decision[point] = 0

			else:
				# classify data groups

				if not group_signs:
					# Make a sign object
					group_signs.append(Sign(num_sign,point,distance,s))

				else:
					if abs(point[0]-group_signs[-1].lastPoint[0])> grouping_rule[0]:
						num_sign+=1
						group_signs.append(Sign(num_sign,point,distance,s))
					else:
						if abs(group_signs[-1].distance-distance)<grouping_rule[1]:
							group_signs[-1].update(point,distance,s)
						else:

							found_in_group = False
							for j in range(len(group_signs)):
								if abs(group_signs[j].distance-distance)<grouping_rule[1]:
									group_signs[j].update(point,distance,s)
									found_in_group = True
									break

							if not found_in_group:
								num_sign+=1
								group_signs.append(Sign(num_sign,point,distance,s))

			i+=1

		for ii in range(len(group_signs)):
			print(round(group_signs[ii].distance/1000,3))

	return r_decision, group_signs

def filterByAzimuthAndBeam(frame,azimuth_range:'list',beam_range:'list'):
	##
	# One frame is a 360 degree scan of the surroundings. VLP16 is at 0.1s rate, which means one frame per 0.1s.
	# Azimuth is n x 1 array where n is the size of filtered azimuth points per frame, n = 24 x range_frame
	# Distance is n x beam_range
	# Reflectivity is n x beam_range

	range_frame =[0,0]
	azimuth_upper=[]
	azimuth_lower=[]

	azimuth = []
	distance = []
	reflectivity = []
	altitude = []

	# Filter frame by azimuth angle
	# Only works for range counter from 90 to 90 degree
	
	azimuth_first = ((frame[3]<<8)| frame[2])/100
	azimuth_last = ((frame[-103]<<8)| frame[-104])/100

	azimuth_first = azimuth_first+360 if azimuth_first>=0 and azimuth_first<=180 else azimuth_first
	if (azimuth_range[1]>azimuth_range[0] and azimuth_range[1]<azimuth_first) or ((azimuth_last+360 - azimuth_first) < 5):
		print('Error!!!Throw the frame!')
		return 0,0,0
	range_frame[0] = int(0 if azimuth_range[0] < azimuth_first else (azimuth_range[0]-azimuth_first)//azimuthPerMessage)
	range_frame[1] = int((azimuth_range[1]-azimuth_first)//azimuthPerMessage if azimuth_range[1]>90 else (azimuth_range[1]+360-azimuth_first)//azimuthPerMessage)

	for i in range(range_frame[0],range_frame[1]):
		for j in range(messageBlockSize):
			azimuth_upper.append(((frame[3+(i*1206)+(j*100)] << 8)| frame[2+(i*1206)+(j*100)])/100)

	for i in range(len(azimuth_upper)-1):
		delta = (azimuth_upper[i+1]-azimuth_upper[i]) if azimuth_upper[i+1]>azimuth_upper[i] else (azimuth_upper[i+1]-azimuth_upper[i]+360)
		azimuth_lower_temp = azimuth_upper[i]+(delta)/2
		azimuth_lower_temp = azimuth_lower_temp if azimuth_lower_temp<360 else azimuth_lower_temp-360
		azimuth_lower.append(round(azimuth_lower_temp,2))
	last_azimuth = azimuth_upper[-1]+(delta)/2
	last_azimuth = last_azimuth if last_azimuth<360 else last_azimuth-360
	azimuth_lower.append(round(last_azimuth,2))
	for i in range(len(azimuth_upper)):
		azimuth.append(azimuth_upper[i])
		azimuth.append(azimuth_lower[i])


	# Distance and reflectivity
	for i in range(range_frame[0],range_frame[1]):
		for j in range(messageBlockSize*2):
			d=[]
			r=[]
			for beamid in range(beam_range[0],beam_range[1]): # Beam is sorted from top downwards
				k = 15-beamid*2 if beamid<8 else 15-beamid*2-1 # calculate beam altitude index (hint:-15,1,-13,3,...,-1,15)
				d.append(((frame[k*3+(5+48*(j%2))+(j//2*100)+(i*1206)] << 8) | frame[k*3+(4+48*(j%2))+(j//2*100)+(i*1206)])*2)   # Distance multiply by 2mm
				r.append(frame[k*3+(6+48*(j%2))+(j//2*100)+(i*1206)])
			distance.append(d)	
			reflectivity.append(r)

	return np.asarray(azimuth), np.asarray(distance), np.asarray(reflectivity)

def filterByDistance(distance_array,distance_threshold):
	
	idx_tuple_d = np.where(distance_array<distance_threshold[0]*1000)
	return idx_tuple_d

def filterByXYZ(x_array,y_array,z_array,xyz_threshold):
	dim = x_array.shape
	xy_decision = np.zeros(dim)
	xyz_decision = np.zeros(dim)

	for i in range(dim[1]-1):
		# Find index where x is non-zero
		idx_tuple_nonzero = np.where(x_array[:,i] != 0)

		if idx_tuple_nonzero:
			x_wise_decision = np.isclose(x_array[idx_tuple_nonzero,i+1],x_array[idx_tuple_nonzero,i],0,xyz_threshold[0])
			xy_decision[idx_tuple_nonzero,i] = x_wise_decision
			xy_decision[idx_tuple_nonzero,i+1] = x_wise_decision

	# index = np.where(xy_decision==1)
	# index_XYZ = np.nonzero(xy_decision)
	return xy_decision

def getCoordinates(azimuth,distance,beam_range):
	altitude = []
	for i in range(beam_range[0],beam_range[1]):
		altitude.append(15-i*2)
	altitude = np.asarray(altitude)

	if azimuth.shape[0] != distance.shape[0]:
		# Error handling method
		print('Error!!!')
		return 0

	projection = distance*(np.cos(np.radians(altitude)))
	z = distance*(np.sin(np.radians(altitude)))
	x = np.transpose(np.transpose(projection)*np.sin(np.radians(azimuth)))
	y = np.transpose(np.transpose(projection)*np.cos(np.radians(azimuth)))

	return x,y,z

def visualize3D(coordinates, index, frame_counter):
	plt.waitforbuttonpress()
	plt.cla()
	# plt.pause(1)
	# print(coordinates[0][index],'\n')
	# print(coordinates[1][index],'\n')
	ax.scatter(coordinates[0][index]/1000,coordinates[1][index]/1000,coordinates[2][index]/1000, c='r', marker='.', s=20)
	# ax.scatter(xp,yp,zp, c='b', marker='.', s=10)
	ax.set_title('Frame: ' + str(frame_counter))
	ax.set_xlim3d([-20, 20])
	ax.set_xlabel('X')
	ax.set_ylim3d([0, 30])
	ax.set_ylabel('Y')
	ax.set_zlim3d([0, 8])
	ax.set_zlabel('Z')
	plt.draw()

def visualizeSign(coordinates, index, frame_counter, group:'list'):
	plt.waitforbuttonpress()
	plt.cla()

	ax.scatter(coordinates[0][index]/1000,coordinates[1][index]/1000,coordinates[2][index]/1000, c='r', marker='.', s=5)
	
	for i in range(len(group)):
		center = group[i].center
		x,y,z=coordinates[0][center]/1000,coordinates[1][center]/1000,coordinates[2][center]/1000
		ax.scatter(x,y,z, c='k', marker='x', s=30)
		text = 'Sign '+str(group[i].name)+', '+str(round(group[i].distance/1000,2))+'m'
		ax.text(x,y,z, text, size=10, zorder=1, color='k') 

	# ax.scatter(xp,yp,zp, c='b', marker='.', s=10)
	ax.set_title('Frame: ' + str(frame_counter))
	ax.set_xlim3d([-20, 20])
	ax.set_xlabel('X')
	ax.set_ylim3d([0, 30])
	ax.set_ylabel('Y')
	ax.set_zlim3d([0, 8])
	ax.set_zlabel('Z')

	plt.draw()

def resetFrameBuffer():
	frameBuffer = bytearray()
	frameSampleSize = 0
	return frameBuffer, frameSampleSize

# VLP device definition
scanSpeed = 600 # RPM
messageBlockSize = 12
azimuthPerMessage = 4.8 #4.8 degree per 12 block data @ 600 RPM
beamSize = 16
beamLayer = [-15,1,-13,3,-11,5,-9,7,-7,9,-5,11,-3,13,-1,15]
detectionQueue = deque(8*[0], 8) # queue to detect the data frame
UDPportSearchArray = [b'\x09', b'\x40', b'\x09', b'\x40', b'\x04', b'\xbe', b'\x00', b'\x00']
LastItemInSearch = b'\x00'


# Filter initialization
beamToKeep = [0,8] # Count from top beam downwards
azimuthToKeep = [270, 90]

# Sign detection
_traffic_sign_filter_criteria = [100,50,8,1,6] # Retro reflection threshold, minimum distance, pointcloud row, column, retain criteria
_traffic_sign_grouping_rule = [50,2000] # Index separation from previous point; Difference of runing calculation of average distance of point cloud


# Object detection
xyzDistThres = [25,25,25] # Corresponding to X Y Z in mm
distThres = [25000] # Corresponding to d value in mm
coordiThres = [50,50,50] # Corresponding to X Y Z in mm


# Processing indication
msgTotal = 0
frmCounter = 0
timestamp = []


#######################################################################

frm, frmSize = resetFrameBuffer()
plt.ion()
fig = plt.figure()	
ax = fig.add_subplot(111, projection='3d')

# Open file
filename = "C:/Users/hux/Desktop/Lidar/Data/test.pcap"
# SignFilterTesting1
# ScenarioExitHighway
# ScenarioMergeToHighway
# ScenarioThroughTrafficAndSign
# HighWayOverheadSigns1
# HighWayRoadsideSigns1
# test2

with open(filename, "rb") as binaryData:
	byte = binaryData.read(1)
	while byte != b"": # or just while byte:
		byte = binaryData.read(1)
		detectionQueue.append(byte)

		# Find the UDP message contains required information
		if byte == LastItemInSearch and list(detectionQueue) == UDPportSearchArray:			
			frame_temp = binaryData.read(1206)
			msgTotal += 1

			azimuth_first = ((frame_temp[3]<<8)| frame_temp[2])/100
			azimuth_last = ((frame_temp[1103]<<8)| frame_temp[1102])/100
			timestamp_temp = (frame_temp[1203]<<24 | frame_temp[1202]<<16 | frame_temp[1201]<<8 | frame_temp[1200])/1000000

			frm.extend(frame_temp)
			frmSize += 1

			# Construct raw frame which persist all data (Raw frame is from 90 to 90 degree)
			if azimuth_first > 180 and azimuth_first < (180+azimuthPerMessage):
				frmCounter += 1
				timestamp.append(timestamp_temp)
				print('Frm: '+str(frmCounter)+', T: '+str(round(timestamp_temp,2))+'s')
				# print(frmSize)
				# print('Contains '+str(frmSize)+' Messages')
				if frmSize >= 360/azimuthPerMessage/2:

					a,d,r = filterByAzimuthAndBeam(frm, azimuthToKeep, beamToKeep)
					
					x,y,z = getCoordinates(a,d,beamToKeep)

					r_decision,signs = filterByReflectivity(r,d,_traffic_sign_filter_criteria,_traffic_sign_grouping_rule)

					index = np.where(r_decision==1)

					# index = np.where(np.logical_xor(r_decision,xyz_decision)==1)


					visualizeSign([x,y,z],index,frmCounter,signs)


					# xyz_decision = filterByXYZ(x,y,z,coordiThres)

					# index2 = filterByDistance(d,distThres)

					# visualize3D([x,y,z],index,frmCounter)


					# Reset message buffer after data being processed
					frm, frmSize = resetFrameBuffer()
				else:
					print('Frm: '+str(frmCounter)+' with '+str(frmSize)+' messages is dumped')

print('Total Counter of Frames and Messages: '+str(frmCounter)+', '+str(msgTotal))
plt.show(block=True)



def main():
	print('hahaha')

if __name__ == "__main__":
    main()

