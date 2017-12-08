#!/usr/bin/env python
import rospy
import std_msgs
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from lab4_cam.srv import ImageSrv, ImageSrvResponse
from lab4_cam.srv import CamInfoSrv, CamInfoSrvResponse
import cv2, time, sys
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from numpy.linalg import *
from sensor_msgs.msg import CameraInfo
import image_geometry as ig
import tf, tf2_ros
from visualization_msgs.msg import Marker, MarkerArray
# Create a CvBridge to convert ROS messages to OpenCV images
bridge = CvBridge()
camInfo = None
def camInfoGet(message):
	camInfo = message
# Converts a ROS Image message to a NumPy array to be displayed by OpenCV
def ros_to_np_img(ros_img_msg):
	return np.array(bridge.imgmsg_to_cv2(ros_img_msg,'bgr8'))
# Increases saturation/value
def change_hsv(img, is_value=True, value=100):
	h, s, v = cv2.split(img)
	lim = 255 - value
	if is_value:
		v[v > lim] = 255
		v[v <= lim] += value
	else:
		s[s > lim] = 255
		s[s <= lim] += value
	hsv = cv2.merge((h, s, v))
	return hsv

# Gets transform matrix
def get_matrix(tf_listener, dest_frame, source_frame):
	try:		
		tf_listener.waitForTransform(dest_frame, source_frame, rospy.Time(), rospy.Duration(2.0))
		tf_listener.waitForTransform(dest_frame, source_frame, rospy.Time(0), rospy.Duration(2.0))
		trans, rot = tf_listener.lookupTransform(dest_frame, source_frame, rospy.Time(0))
		matrix = listener.fromTranslationRotation(trans, rot)
		return matrix
	except tf.LookupException as e:
		print(e)
		print("get_matrix error tf")
		return None
	except tf2_ros.TransformException as e:
		print(e)
		print("get_matrix error tf2")
		return None

def LinePlaneCollision(planeNormal, planePoint, rayDirection, rayPoint, epsilon=1e-6):
	ndotu = planeNormal.dot(rayDirection)
	if abs(ndotu) < epsilon:
		print("no intersection")
		return None
	w = rayPoint - planePoint
	si = -planeNormal.dot(w) / ndotu
	Psi = w + si * rayDirection + planePoint
	return Psi

if __name__ == '__main__':
	rospy.wait_for_service('last_image')
	rospy.wait_for_service('last_cam_info')
	marker_publisher = rospy.Publisher('cam_marker', MarkerArray, queue_size=10)
	centroid_publisher = rospy.Publisher('centroids2', Point, queue_size=10)

	rospy.init_node('image_processor', anonymous=True)

	last_image_service = rospy.ServiceProxy('last_image', ImageSrv)
	last_cam_info_srv = rospy.ServiceProxy('last_cam_info', CamInfoSrv)
	listener = tf.TransformListener()
	rate = rospy.Rate(10.0)
	head_to_base = None
	ar_to_base = None
	base_to_ar = None
	while not rospy.is_shutdown():
		try:
			ros_img_msg = last_image_service().image_data
			ros_cam_info = last_cam_info_srv().cam_info 
			np_image = ros_to_np_img(ros_img_msg)

			headcam = ig.PinholeCameraModel()
			headcam.fromCameraInfo(ros_cam_info)

			hue = 70
			hue_range = 20
			blur = cv2.medianBlur(np_image, 9)
			hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
			hsv = change_hsv(hsv, is_value=True)
			hued = cv2.inRange(hsv, (hue-hue_range, 80, 80), (hue+hue_range, 255, 255))
			_, contours, hierarchy = cv2.findContours(hued, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
			thresh = 25
			filtered = [cnt for cnt in contours if cv2.contourArea(cnt) > thresh]

			hb = get_matrix(listener, "base", "head")
			if hb is not None and hb.any():
				head_to_base = hb
			ab = get_matrix(listener, "base", "ar_marker_4")
			if ab is not None and ab.any():
				ar_to_base = ab
			ba = get_matrix(listener, "ar_marker_4", "base")
			if ba is not None and ba.any():
				base_to_ar = ba


			if head_to_base is None or ar_to_base is None or base_to_ar is None:
				continue
			origin = np.array([0, 0, 0, 1])
			z_axis = np.array([0, 0, 1, 1])
			head_origin = head_to_base.dot(origin)
			ar_origin = ar_to_base.dot(origin)
			ar_normal = ar_to_base.dot(z_axis)
			in_frame = []
			for contour in filtered:
				M = cv2.moments(contour)
				cX = int(M["m10"] / M["m00"])
				cY = int(M["m01"] / M["m00"])

				ray = headcam.projectPixelTo3dRay(headcam.rectifyPoint((cX, cY)))

				point_in_base = head_to_base.dot(np.array([ray[0], ray[1], ray[2], 1]))
				p = Point()

				raydir = point_in_base - head_origin
				planedir = ar_normal - ar_origin
				planed = LinePlaneCollision(planedir[0:3], ar_origin[0:3], raydir[0:3], point_in_base[0:3])
				point_in_ar = base_to_ar.dot(np.array([planed[0], planed[1], planed[2], 1]))
				if point_in_ar[0] < 0.60 and point_in_ar[0] > -0.04:
					if point_in_ar[1] < 0.45 and point_in_ar[1] > -0.04:
						point_in_ar[2] += 0.20
						final = ar_to_base.dot(point_in_ar)
						p.x = point_in_ar[0]
						p.y = point_in_ar[1]
						p.z = point_in_ar[2]
						centroid_publisher.publish(p)
						# print final
						in_frame.append(contour)
						cv2.circle(np_image, (cX, cY), 1, (0, 0, 255), 2)
			cv2.drawContours(np_image, in_frame, -1, (0, 255, 0), 2)
			cv2.imshow("test", np_image)
			cv2.waitKey(1000)

		except KeyboardInterrupt:
			print("keyboard interrupt, exiting")
			break

		except rospy.ServiceException as e:
			print(e)
			break
	cv2.destroyAllWindows()