import rospy
import intera_interface
import intera_external_devices

def main():
	limb = intera_interface.Limb('right')


	joints = limb.joint_names()

	done = False
	while not done and not rospy.is_shutdown():
		c = intera_external_devices.getch()
		if c:
			#catch Esc or ctrl-c
			if c in ['\x1b', '\x03']:
				done = True
				rospy.signal_shutdown("Finished.")
		
		def set_done(angles):
			_done = True
			cur_angles = limb.joint_angles()
			if abs(cur_angles[joints[6]] - angles[joints[6]]) > 0.1:
					_done = False

			return _done



		angles = limb.joint_angles()

		angles[joints[6]] = 0		
		while not set_done(angles):
		   limb.set_joint_positions(angles)

		angles[joints[6]] = 3.14
		while not set_done(angles):
		   limb.set_joint_positions(angles)

		angles[joints[6]] = 0
		while not set_done(angles):
		   limb.set_joint_positions(angles)


		done = True
			

if __name__ == '__main__':
	main()
