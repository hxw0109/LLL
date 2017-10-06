import numpy as np
from collections import deque
import math
x = np.zeros((2,10,4))
x[0,0,0] = 1
x[0,0,1] = 2
x[0,0,2] = 3
x[0,0,3] = 4



b = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]
# print (b[-4])

a = []
a.append(3)
a.append(4)


queue = deque(8*[0], 8)
queue.append(4)
queue.append(6)
queue.append(7)
queue.append(8)
queue.append(9)
queue.append(10)
queue.append(11)
queue.append(14)
queue.append(20)
queue.append(21)

a.append([3,4])

x = np.array([[1, 2, 3],[5,6,7]])
dd = np.array([1, 2, 3])
m = np.asmatrix(x)
a = np.cos(np.radians(x))
# print(dd.shape)

# x = np.arange(9.).reshape(3, 3)
# print(x)
# print(np.where( x > 5 ))


a = np.zeros((3,4))

b = np.ones((3,4))


b[0,2]=0
b[2,3] =0
a[1,1] =1
a[1,2] =1



x = np.array([[1, 2, 3],[0,6,7],[3,4.1,6],[2,1.9,34]])
xnew = np.zeros(x.shape)

index = np.where(x[:,0] != 0)
index_m = np.where(x>3)

decision = np.isclose(x[index,1],x[index,0],0,1)

xnew[index,0] = decision
xnew[index,1] = decision

# print(x)
# print(index_m)
idx = list(zip(index_m[0],index_m[1]))[4]
a = [1, 1, 2, 2, 3]
b = [1, 2, 1, 2, 2]
c =[a,b]
# print(x)
# print(x[0:0:1,4:7:1])
# print(type(x))


class DecisionMatrix(object):
	def __init__(self,data_matrix,origin,shape):
		self.data = data_matrix
		self.origin = origin
		self.shape = shape
		self.offset = [0,0]

	def offsets(self,offset):
		self.offset = [offset[0],offset[1]]

	def m_with_offset(self):
		self.m = self.data[(self.origin[0]+self.offset[0]):
			(self.origin[0]+self.offset[0]+self.shape[0]):1, 
			(self.origin[1]+self.offset[1]):
			(self.origin[1]+self.offset[1]+self.shape[1]):1]
		return self.m

	# def decision(self):
	# 	decision = True if sum(sum(self.m())) >= self.criteria else False
	# 	return decision

	def counter(self):
		counter = sum(sum(self.m))
		return counter

# a = np.zeros((3,4))

# b = np.ones((3,4))
# a[1,2] = 1
# a[2,3] = 1
# b[0,0] = 0
# b[2,3] = 0
# print(a,'\n\n',b,'\n',np.logical_xor(a,b))


dict = dict()
dict[1] = {'df':[]}
dict[1]['df'].append(2)


print(x)
a = [1,2]
print(type(a))
b = tuple(a)
print(type(b))
print(b[0],b[1])

print(17//2)

