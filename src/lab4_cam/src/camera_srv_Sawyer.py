#!/usr/bin/env python
import rospy

from sensor_msgs.msg import Image, CameraInfo
from lab4_cam.srv import ImageSrv, ImageSrvResponse
from lab4_cam.srv import CamInfoSrv, CamInfoSrvResponse


class ImgService:
  #Callback for when an image is received
  def imgReceived(self, message):
    #Save the image in the instance variable
    self.lastImage = message

    #Print an alert to the console
    #print(rospy.get_name() + ":Image received!")

  def camInfoReceived(self, message):
    self.lastCamInfo = message
  #When another node calls the service, return the last image
  def getLastImage(self, request):
    #Print an alert to the console
    #print("Image requested!")

    #Return the last image
    return ImageSrvResponse(self.lastImage)

  def getLastCamInfo(self, request):
    return CamInfoSrvResponse(self.lastCamInfo)
  def __init__(self):
    #Create an instance variable to store the last image received
    self.lastImage = None
    self.lastCamInfo = None
    #Initialize the node
    rospy.init_node('cam_listener')

    #Subscribe to the image topic
    rospy.Subscriber("/io/internal_camera/head_camera/image_raw", Image, self.imgReceived)
    rospy.Subscriber("/io/internal_camera/head_camera/camera_info", CameraInfo, self.camInfoReceived)
    #Create the service
    rospy.Service('last_image', ImageSrv, self.getLastImage)
    rospy.Service('last_cam_info', CamInfoSrv, self.getLastCamInfo)
  def run(self):
    rospy.spin()

#Python's syntax for a main() method
if __name__ == '__main__':
  node = ImgService()
  node.run()
