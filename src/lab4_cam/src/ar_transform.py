import rospy
import tf
from std_msgs import Header
from ar_track_alvar import AlvarMarkers, AlvarMarker
from geometry_msgs import Pose, PoseStamped, Point, Quaternion

def listener():
	rospy.init_node('ar_marker_frame')
	rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback)

def get_transform(frame1, frame2):
  listener = tf.TransformListener()
  rate = rospy.Rate(10.0)
  if (listener.waitForTransform(frame1, frame2, rospy.Time(), rospy.Duration(4.0)) or 
  	 listener.waitForTransform(frame1, frame2, rospy.Time(0), rospy.Duration(4.0))):
  	translation, rotation = listener.lookupTransform(frame1, frame2, rospy.Time(0))
  	matrix = listener.fromTranslationRotation(translation, rotation)
  	return matrix
  else:
  	println("No transform found between " + frame1 + " and " + frame2 + ".")
  	return

def callback(message):	
if __name__ == '__main__':
	while not rospy.is_shutdown():
		listener = tf.TransformListener()
		rate = rospy.Rate(10.0)
		if listener.waitForTransform()
			# for marker in message.markers:
		# 	print(marker.id)
		# 	if (marker.id == "ar_marker_13"):
		# 		ar_id = marker.id
		# 		pos = message.markers[0].pose.pose.position
		# 		ori = message.markers[0].pose.pose.orientation
		# 		ar_position = (pos.x, pos.y, pos.z)
		# 		ar_orientation = (ori.x, ori.y, ori.z, ori.w)


		# br = tf.TransformBroadcaster()
		# rate = rospy.Rate(10.0)
		# while not rospy.is_shutdown():
		# 	br.sendTransform((ar_position),
		# 					 (ar_orientation),
		# 					 rospy.Time.now(),
		# 					 "base",
		# 					 ar_id + "_frame")
		# 	rate.sleep()