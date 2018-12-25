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
			return G / distance ** 2
		else:
			#Collision
			return - G / distance ** 3

	#Vector force induced by other point
	def force(self, other: 'MatPoint')-> 'numpy.array':
		force = numpy.array([0, 0])
		dr = other.position - self.position
		distance = numpy.linalg.norm(dr)
		if (distance == 0):
			#Dencity with the same point
			return force
		force = dr * other.mass * self.mass * self.dencity(distance) / distance
		return force

	def apply_force(self, force: 'numpy.array'):
		self.velocity += force / self.mass * delta_t

	#Move the point on delta_t
	def move(self):
		self.position += self.velocity * delta_t

#Count of moveable points
points_count = 3
#Moveable points
points = []

#Define points start positions, velocities and masses
def init():
	Masses = [2.0, 10.0, 1.0]
	X = [0.0, 0.0, 0.0]
	Y = [15.0, 0.0, -10.0]
	velocities_X = [5.0, 0.0, -10.0]
	velocities_Y = [0.0, 0.0, 0.0]
	for i in range(0, points_count):
		points.append(MatPoint(Masses[i], numpy.array([X[i], Y[i]]), numpy.array([velocities_X[i], velocities_Y[i]])))

init()

positions_X = []
positions_Y = []

for numpoint in range(0, points_count):
	positions_X.append([])
	positions_Y.append([])

#Move

count_of_steps = int(all_time / delta_t)

for n in range(0, count_of_steps):
	for point in points:
		point.move()
		for other in points:
			point.apply_force(point.force(other))
	for numpoint in range(0, points_count):
		positions_X[numpoint].append(points[numpoint].position[0])
		positions_Y[numpoint].append(points[numpoint].position[1])

#Draw

for numpoint in range(0, points_count):
	plt.plot(positions_X[numpoint], positions_Y[numpoint])

plt.show()
