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
import tf
from visualization_msgs.msg import Marker, MarkerArray
# Nominal length of a tile side
TILE_LENGTH = 30.48 #cm

# Helper function to check computed homography
# This will draw dots in a grid by projecting x,y coordinates
# of tile corners to u,v image coordinates
def check_homography(image, H, nx, ny, length=TILE_LENGTH):
  # H should be a 3x3 numpy.array
  # nx is the number of tiles in the x direction
  # ny is the number of tiles in the y direction
  # length is the length of one side of a tile
  # image is an image array
  for i in range(nx+1):
    for j in range(ny+1):
      xbar = np.array([[i*length],[j*length],[1]])
      ubar = np.dot(H,xbar).T[0]
      u = np.int(ubar[0]/ubar[2])
      v = np.int(ubar[1]/ubar[2])
      print 'Dot location: ' + str((u,v))
      cv2.circle(image, (u,v), 5, 0, -1)
  cv2.imshow('Check Homography', image)

# Create a CvBridge to convert ROS messages to OpenCV images
bridge = CvBridge()
camInfo = None
def camInfoGet(message):
  camInfo = message
# Converts a ROS Image message to a NumPy array to be displayed by OpenCV
def ros_to_np_img(ros_img_msg):
  return np.array(bridge.imgmsg_to_cv2(ros_img_msg,'bgr8'))

# Define the total number of clicks we are expecting (4 corners)
TOT_CLICKS = 4

if __name__ == '__main__':

  # Waits for the image service to become available
  rospy.wait_for_service('last_image')
  rospy.wait_for_service('last_cam_info')
  pub = rospy.Publisher('cam_marker', MarkerArray, queue_size=10)

  pub2 = rospy.Publisher('centroids', Point, queue_size=10)

  # Initializes the image processing node
  rospy.init_node('image_processing_node')

  # Creates a function used to call the
  # image capture service: ImageSrv is the service type
  last_image_service = rospy.ServiceProxy('last_image', ImageSrv)
  last_cam_info_srv = rospy.ServiceProxy('last_cam_info', CamInfoSrv)
  # Create an empty list to hold the coordinates of the clicked points
  points = []

  # Callback function for 'cv2.SetMouseCallback' adds a clicked point to the
  # list 'points'
  def on_mouse_click(event,x,y,flag,param):
    if(event == cv2.EVENT_LBUTTONUP):
      point = (x,y)
      print "Point Captured: " + str(point)
      points.append(point)

  while not rospy.is_shutdown():
    try:
      # Waits for a key input to continue
      raw_input('Press enter to capture an image:')
    except KeyboardInterrupt:
      print 'Break from raw_input'
      break

    try:
      # Request the last image from the image service
      # And extract the ROS Image from the ImageSrv service
      # Remember that ImageSrv.image_data was
      # defined to be of type sensor_msgs.msg.Image
      ros_img_msg = last_image_service().image_data
      ros_cam_info = last_cam_info_srv().cam_info
      # Convert the ROS message to a NumPy image
      np_image = ros_to_np_img(ros_img_msg)


      listener = tf.TransformListener()
      rate = rospy.Rate(10.0)
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
      
      # Display the CV Image
      headCam = ig.PinholeCameraModel()
      headCam.fromCameraInfo(ros_cam_info)
      hue = 70 #70 for green
      hue_range = 20
      blur = cv2.medianBlur(np_image, 9)
      hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
      hsv = change_hsv(hsv, is_value=True)
      hued = cv2.inRange(hsv, (hue-hue_range, 80, 80), (hue+hue_range, 255, 255))
      contours, hierarchy = cv2.findContours(hued, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
      thresh = 25
      filtered = [cnt for cnt in contours if cv2.contourArea(cnt) > thresh]
      cv2.drawContours(np_image, filtered, -1, (0, 255, 0), 2)

      # cv2.imshow("saturated", cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR))

      listener.waitForTransform("base", "head_camera", rospy.Time(), rospy.Duration(4.0))
      listener.waitForTransform("base", "head_camera", rospy.Time(0), rospy.Duration(4.0))
      translation, rotation = listener.lookupTransform("base", "head_camera", rospy.Time(0))
      matrix = listener.fromTranslationRotation(translation, rotation)

      listener.waitForTransform("base", "ar_marker_4", rospy.Time(), rospy.Duration(4.0))
      listener.waitForTransform("base", "ar_marker_4", rospy.Time(0), rospy.Duration(4.0))

      listener.waitForTransform("ar_marker_4", "base", rospy.Time(), rospy.Duration(4.0))
      listener.waitForTransform("ar_marker_4", "base", rospy.Time(0), rospy.Duration(4.0))

      translation2, rotation2 = listener.lookupTransform("base", "ar_marker_4", rospy.Time(0))
      matrix2 = listener.fromTranslationRotation(translation2, rotation2)

      translation3, rotation3 = listener.lookupTransform("ar_marker_4", "base", rospy.Time(0))
      matrix3 = listener.fromTranslationRotation(translation3, rotation3)

      origin = np.array([0, 0, 0, 1])
      z_axis = np.array([0, 0, 1, 1])
      head_origin = matrix.dot(origin)
      ar_origin = matrix2.dot(origin)
      ar_normal = matrix2.dot(z_axis)
      def LinePlaneCollision(planeNormal, planePoint, rayDirection, rayPoint, epsilon=1e-6):
        ndotu = planeNormal.dot(rayDirection)
        if abs(ndotu) < epsilon:
          raise RuntimeError("no intersection or line is within plane")
        w = rayPoint - planePoint
        si = -planeNormal.dot(w) / ndotu
        Psi = w + si * rayDirection + planePoint
        return Psi
      marker_array = MarkerArray()
      i = 0
      for contour in filtered:
          M = cv2.moments(contour)
          cX = int(M["m10"] / M["m00"])
          cY = int(M["m01"] / M["m00"])
          cv2.circle(np_image, (cX, cY), 1, (0, 0, 255), 2)
          ray = headCam.projectPixelTo3dRay(headCam.rectifyPoint((cX, cY)))
          # print headCam.rectifyPoint((cX, cY))
          point_in_base = matrix.dot(np.array([ray[0], ray[1], ray[2], 1]))

          p = Point()

          test_marker = Marker()
          test_marker.header.frame_id = "ar_marker_4"
          test_marker.header.stamp = rospy.get_rostime()
          test_marker.pose.position = p
          test_marker.id = i
          test_marker.type = 2
          test_marker.pose.orientation.x = 0
          test_marker.pose.orientation.z = 0
          test_marker.pose.orientation.w = 1.0
          test_marker.scale.x = 0.1
          test_marker.scale.y = 0.1
          test_marker.scale.z = 0.1

          test_marker.color.r = 0.0
          test_marker.color.g = 1.0
          test_marker.color.b = 0.0
          test_marker.color.a = 1.0
          raydir = point_in_base - head_origin
          # print raydir[0:3]
          planedir = ar_normal - ar_origin
          planed_point = LinePlaneCollision(planedir[0:3], ar_origin[0:3], raydir[0:3], point_in_base[0:3])
          planed_point2 = np.array([planed_point[0], planed_point[1], planed_point[2], 1])
          planed_point2_in_ar = matrix3.dot(planed_point2)
          if planed_point2_in_ar[0] < 0.60 and planed_point2_in_ar[0] > -0.04:
            if planed_point2_in_ar[1] < 0.45 and planed_point2_in_ar[1] > -0.04:
              planed_point2_in_ar[2] += 0.20
              final = matrix2.dot(planed_point2_in_ar)
              p.x = planed_point2_in_ar[0]
              p.y = planed_point2_in_ar[1]
              p.z = planed_point2_in_ar[2]
              pub2.publish(p)
              i += 1
              print final
              marker_array.markers.append(test_marker)
      pub.publish(marker_array)
      cv2.imshow("test", np_image)
      # Tell OpenCV that it should call 'on_mouse_click' when the user
      # clicks the window. This will add clicked points to our list
      cv2.setMouseCallback("CV Image", on_mouse_click, param=1)

      # Zero out list each time we have a new image
      points = []
      # Loop until the user has clicked enough points
      while len(points) < TOT_CLICKS:
        if rospy.is_shutdown():
          raise KeyboardInterrupt
        cv2.waitKey(10)

      # Convert the Python list of points to a NumPy array of the form
      #   | u1 u2 u3 u4 |
      #   | v1 v2 v3 v4 |
      uv = np.array(points).T

# === YOUR CODE HERE ===========================================================

      # This is placeholder code that will draw a 4 by 3 grid in the corner of
      # the image
      nx = 4
      ny = 3
      H = np.eye(3)

# ==============================================================================

      # Check the produced homography matrix
      check_homography(np_image, H, nx, ny)

      # Loop until the user presses a key
      key = -1
      while key == -1:
        if rospy.is_shutdown():
          raise KeyboardInterrupt
        key = cv2.waitKey(100)

      # When done, get rid of windows and start over
      # cv2.destroyAllWindows()

    except KeyboardInterrupt:
      print 'Keyboard Interrupt, exiting'
      break

    # Catch if anything went wrong with the Image Service
    except rospy.ServiceException, e:
      print "image_process: Service call failed: %s"%e

  cv2.destroyAllWindows()
