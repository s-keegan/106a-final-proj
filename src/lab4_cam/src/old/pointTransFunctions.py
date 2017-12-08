import rospy
import numpy as np
from numpy import linalg


def pixelTo2DPoint(pixel, H):
	point = np.dot(pixel,H)
	return point/point[2]

def pixelTo2DPlaneTransform(h, l, tagPixels):

	tl = tagPixels[0]
	bl = tagPixels[1]
	br = tagPixels[2]
	tr = tagPixels[3]

	A = np.array([[0, h, 1, 0, 0, 0, -tl[0]*0, -tl[0]*h],
		[0, 0, 0, 0, h, 1,  -tl[1]*0, -tl[1]*h],
		[0, 0, 1, 0, 0, 0, -bl[0]*0, -bl[0]*0],
		[0, 0, 0, 0, 0, 1,  -bl[1]*0, -bl[1]*0],
		[l, h, 1, 0, 0, 0, -tr[0]*l, -tr[0]*h],
		[0, 0, 0, l, h, 1,  -tr[1]*l, -tr[1]*h],
		[l, 0, 1, 0, 0, 0, -br[0]*l, -br[0]*0],
		[0, 0, 0, l, 0, 1,  -br[1]*l, -br[1]*0]])
	b = np.transpose(np.array([tl[0],tl[1],bl[0],bl[1],tr[0],tr[1],br[0],br[1]]))
	h = np.linalg.lstsq(A, b)
	Htrans = np.array([[h[0],h[1],h[2]],
		[h[3],h[4],h[5]],
		[h[6],h[7],1]])
	return Htrans

def PlaneTo3DPoint(x, y, h, l, points):
	#points - tl, bl, tr, br
	y_prime = (points[0]-points[1])*(y/h)
	x_prime = (points[3]-points[1])*(x/l)
	p = y_prime + x_prime + points[1]

	return p




