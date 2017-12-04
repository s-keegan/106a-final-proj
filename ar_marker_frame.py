import roslib
roslib.load_manifest('learning_tf')

import rospy
import tf
from std_msgs import Header
from ar_track_alvar import AlvarMarkers, AlvarMarker
from geometry_msgs import Pose, PoseStamped, Point, Quaternion

ar_position = (0, 0, 0)
ar_orientation = (0, 0, 0, 0)
ar_id = ''

def listener():
	rospy.init_node('ar_marker_frame')
	rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback)

def callback(message):
	for marker in message.markers:
		print(marker.id)
		if (marker.id == "ar_marker_13"):
			ar_id = marker.id
			pos = message.markers[0].pose.pose.position
			ori = message.markers[0].pose.pose.orientation
			ar_position = (pos.x, pos.y, pos.z)
			ar_orientation = (ori.x, ori.y, ori.z, ori.w)

	br = tf.TransformBroadcaster()
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		br.sendTransform((ar_position),
						 (ar_orientation),
						 rospy.Time.now(),
						 "base",
						 ar_id + "_frame")
		rate.sleep()
	
if __name__ == '__main__':
	listener()
	