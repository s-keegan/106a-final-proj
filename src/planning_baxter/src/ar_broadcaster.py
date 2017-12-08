#!/usr/bin/env python
import rospy
import numpy as np
import tf, tf2_ros

if __name__ == '__main__':
	rospy.init_node('ar_broadcaster')
	listener = tf.TransformListener()
	br = tf.TransformBroadcaster()
	rate = rospy.Rate(10.0)
	ar_b_trans, ar_b_rot = None, None
	while not rospy.is_shutdown():
		try:
			listener = tf.TransformListener()
			# listener.waitForTransform('base', 'ar_marker_4', rospy.Time(0), rospy.Duration(4.0))
			listener.waitForTransform('base', 'ar_marker_4', rospy.Time(0), rospy.Duration(4.0))
			# b_ar_trans, b_ar_rot = listener.lookupTransform('base', 'ar_marker_4', rospy.Time(0))
			ar_b_trans, ar_b_rot = listener.lookupTransform('base', 'ar_marker_4', rospy.Time(0))
			# br.sendTransform(b_ar_trans, b_ar_rot, rospy.Time(0), "base", "ar_fixed_frame")
			br.sendTransform(ar_b_trans, ar_b_rot, rospy.Time.now(), "ar_fixed_frame", "base")
		except tf.LookupException:
			print("Transform Lookup failed.")
			pass
		except tf2_ros.TransformException:
			print("Transform Lookup failed.")
			pass
