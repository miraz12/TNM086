from H3DInterface import *

class ChangeColor(TypedField(SFColor, MFBool)):
	def update(self, event):
		if len(event.getValue()) > 0 and event.getValue()[0]:
			return RGB(0, 1, 0)
		else:
			return RGB(1, 0, 0)
			
			
changeColor = ChangeColor()