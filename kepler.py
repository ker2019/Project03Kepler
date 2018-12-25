#!/usr/local/bin/python3.6

import matplotlib.pyplot as plt
import numpy
import random
import math

#Little time range
delta_t = 0.01

#gravitational constant
G = 50.0

#Colliding points distance
collision_distance = 0.05

#All time
all_time = 50

#Material point on the flat
class MatPoint:
	def __init__(self, mass: 'float', position: 'numpy.array', velocity: 'numpy.array'):
		self.mass = mass
		self.position = position
		self.velocity = velocity

	#Interaction between points
	@staticmethod
	def dencity(distance: 'float')-> 'float':
		if (distance > collision_distance):
			return G / distance
		else:
			#Collision
			return - G / distance ** 3

	#Vector force induced by other point
	def force(self, other: 'MatPoint')-> 'numpy.array':
		force = numpy.array([0, 0])
		dr = other.position - self.position
		distance = numpy.linalg.norm(dr)
		force = dr * other.mass * self.mass * self.dencity(distance) / distance
		return force

	def apply_force(self, force: 'numpy.array'):
		self.velocity[0] += force[0] / self.mass * delta_t
		self.velocity[1] += force[1] / self.mass * delta_t

	#Move the point on delta_t
	def move(self):
		self.position[0] += self.velocity[0] * delta_t 
		self.position[1] += self.velocity[1] * delta_t

#Count of moveable points
points_count = 1
#Moveable points
points = []

#Fixed point
centrum = MatPoint(10.0, numpy.array([0.0, 0.0]), numpy.array([0.0, 0.0]))

#Define points start positions, velocities and masses
def init():
	for i in range(0, points_count):
		velocity = random.uniform(-2, 2) + 7.0
		points.append(MatPoint(1.0, numpy.array([1.0, 10.0]), 
		numpy.array([velocity, 0.0])))
		print("velocity of", i + 1, "point:", velocity)

init()

positions_X = []
positions_Y = []

for numpoint in range(0, points_count):
	positions_X.append([])
	positions_Y.append([])

#Move

count_of_steps = int(all_time / delta_t)

numpoint = 0
for point in points:
	for n in range(0, count_of_steps):
		positions_X.append(point.position[0])
		positions_Y.append(point.position[1])
		point.move()
		point.apply_force(point.force(centrum))
		positions_X[numpoint].append(point.position[0])
		positions_Y[numpoint].append(point.position[1])
	numpoint += 1

#Draw

c = plt.Circle((0, 0), 1, color='b')
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
ax.add_patch(c)

for numpoint in range(0, points_count):
	plt.plot(positions_X[numpoint], positions_Y[numpoint])

plt.show()
