import rospy
import intera_io as io
import intera_interface
import intera_core_msgs
rospy.init_node("test_commands")
cmd = io.io_interface.IOInterface("/io/internal_camera/head_camera", intera_core_msgs.msg.IOComponentCommand, intera_core_msgs.msg.IODeviceStatus)
print "cmd.publish_command('set_exposure', {})"
