import rospy
import numpy as np
from ar_track_alvar_msgs import AlvarMarkers
from geometry_msgs import PoseStamped

def callback(message):
	markers = msg.markers
	pose = markers[0].pose.pose
	coords = np.array([pose.position.x, pose.position.y, pose.position.z])
	orientation = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

def listener():
	rospy.init.node('ar_pose_reader')
	rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback)