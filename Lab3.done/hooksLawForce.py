from H3DInterface import *
from math import sqrt
from math import cos
from math import sin

class CalcForce(TypedField(SFVec3f, (SFVec3f, MFBool))):
	center = Vec3f(0,0,0)
	rad = 0.05
	di = getActiveDeviceInfo()
	if di:
		hd = di.device.getValue()[0]
	
	def dist(self, deviceX, deviceY, deviceZ):
		return sqrt(pow(deviceX, 2) + pow(deviceY, 2) + pow(deviceZ, 2))
	
	def update(self, event):
		#Spring constant, gradually increase from 100
		k = 400
	
		orientation = self.hd.trackerOrientation.getValue().toEulerAngles()
		offsetx = sin(orientation.x)
		offsety = -(sin(orientation.y)*cos(orientation.x))
		offsetz = -(cos(orientation.y)*cos(orientation.x))
		offset = Vec3f(offsetx, offsety, offsetz) * 0.0
	
		#x = event.getValue().x
		#y = event.getValue().y
		#z = event.getValue().z
		
		x = event.getValue().x + offset.x
		y = event.getValue().y + offset.y
		z = event.getValue().z + offset.z
		
		dist = self.dist(x, y, z)
		
		#If device is inside of sphere
		if dist <= self.rad and dist != 0:
			displacement = self.rad - dist
			forceX = (x/dist) * displacement * k
			forceY = (y/dist) * displacement * k
			forceZ = (z/dist) * displacement * k
			return Vec3f(forceX, forceY, forceZ)
		else:
			return Vec3f(0, 0, 0)
			
calcForce = CalcForce()
			