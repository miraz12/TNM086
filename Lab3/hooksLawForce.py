from H3DInterface import *
from math import sqrt

class CalcForce(TypeField(SFVec3f), (SFVec3f, MFBool)):
	center = Vec3f(0,0,0)
	rad = 0.1
	
	def dist(self, deviceX, deviceY, devicez):
		return sqrt(pow(deviceX, 2) + pow(deviceY, 2) + pow(deviceZ, 2))
	
	def update(self, event);
		#Spring constant, gradually increase from 100
        k = 100
	
		x = event.getValue().x
		y = event.getValue().y
		z = event.getValue().z
		
		dist = self.dist(x, y, z)
		
		#If device is inside of sphere
		if dist <= self.rad and dist != 0:
			displacement = self.rad - dist
			forceX = (x/dist) * displacement * k
			forceY = (x/dist) * displacement * k
			forceZ = (x/dist) * displacement * k
			return Vec3f(forceX, forceY, forceZ)
		else:
			return Vec3f(0, 0, 0)
			
calcForce = CalcForce()
			