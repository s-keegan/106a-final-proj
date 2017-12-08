#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints, CollisionObject, MotionPlanRequest
from geometry_msgs.msg import PoseStamped, TransformStamped, Point
import joint_position_manual as jpm
import numpy as np
import time
from lab4_cam.srv import CentroidSrv,CentroidSrvResponse



def moveit_erase(point3D):
    try:
        #Initialize moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)

        #Start a node
        # rospy.init_node('moveit_node')

        #Initialize arm
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        right_arm = moveit_commander.MoveGroupCommander('right_arm')

        right_arm.set_planner_id('RRTConnectkConfigDefault')
        right_arm.set_planning_time(30)

        #Set the velocity of the joint movement
        right_arm.set_max_velocity_scaling_factor(0.3)

        # Set workspace
        minX = -0.1
        minY = -0.9
        maxX = 1.0
        maxY = 0.6
        right_arm.set_workspace([minX, minY, maxX, maxY])

        #Set joint targets for start position
        start_pos = {
            'right_j0': -.1015,
            'right_j1': -1.25506,
            'right_j2': -2.936244140625,
            'right_j3': -2.6185,
            'right_j4': 0.354,
            'right_j5': 1.2513,
            'right_j6': -4.71146
            }

        start_pos_down = {
            'right_j0': -.1015,
            'right_j1': -1.25506,
            'right_j2': -2.936244140625,
            'right_j3': -2.6185,
            'right_j4': 0.354,
            'right_j5': -0.2446787,
            'right_j6': -4.71146
            }



        #Set plane collision objects
        p = PoseStamped()
        scene.add_plane("left", p, normal = (0.0, 1.0, 0.0), offset = -0.6)    
        scene.add_plane("right", p, normal = (0.0, -1.0, 0.0), offset = -0.6)    
        scene.add_plane("wall", p, normal = (-1.0, 0.0, 0.0), offset = -0.1)
        scene.add_plane("wallfront", p, normal = (1.0, 0.0, 0.0), offset = -1.0)
        scene.add_plane("ceil", p, normal = (0.0, 0.0, 1.0), offset = -1.5)    
        scene.add_plane("floor", p, normal = (0.0, 0.0, 1.0), offset = 0)



        #Set start start_pos as joint target ------------------------------------------
        right_arm.set_joint_value_target(start_pos)

        #Plan a path
        right_planstart = right_arm.plan()

        #Execute the plan
        raw_input('Press <Enter> to move the right arm to start pose.')
        right_arm.execute(right_planstart)



        # rospy.wait_for_service('last_centroids')

        # findNextPoint = rospy.ServiceProxy('last_centroids',CentroidSrv)

        # point3D = findNextPoint().centroid

        # print(point3D)

        # if (point3D.x == 0.0 and point3D.y == 0.0 and point3D.z == 0.0):
        #     print('kdjfsd')
        #     break





        # Move towards board ------------------------------------------------------

        goal_1 = PoseStamped()
        goal_1.header.frame_id = "ar_marker_4"

        #x, y, and z position
        goal_1.pose.position.y = point3D.y
        goal_1.pose.position.x = point3D.x
        goal_1.pose.position.z = .20
        #Orientation as a quaternion
        goal_1.pose.orientation.x = 0.0
        goal_1.pose.orientation.y = -1.0
        goal_1.pose.orientation.z = 0.0
        goal_1.pose.orientation.w = 0.0

        #Set the goal state to the pose you just defined
        right_arm.set_pose_target(goal_1)



        #Set the start state for the right arm
        right_arm.set_start_state_to_current_state()


        #Plan a path
        right_plan = right_arm.plan()

        #Execute the plan
        raw_input('Press <Enter> to move the right arm to move toward board.')
        right_arm.execute(right_plan)


        # # # Uncomment for painting ------------------------------------------------------

        # goal_2 = PoseStamped()
        # goal_2.header.frame_id = "ar_marker_4"

        # #x, y, and z position
        # goal_2.pose.position.y = point3D.y+.04
        # goal_2.pose.position.x = point3D.x
        # goal_2.pose.position.z = .31
        
        # #Orientation as a quaternion
        # goal_2.pose.orientation.x = 0.0
        # goal_2.pose.orientation.y = -1.0
        # goal_2.pose.orientation.z = 0.0
        # goal_2.pose.orientation.w = 0.0

        # orien_const = OrientationConstraint()
        # orien_const.link_name = "right_gripper";
        # orien_const.header.frame_id = "ar_marker_4";
        # orien_const.orientation.y = -1.0;
        # orien_const.absolute_x_axis_tolerance = 0.1;
        # orien_const.absolute_y_axis_tolerance = 0.1;
        # orien_const.absolute_z_axis_tolerance = 0.1;
        # orien_const.weight = 1.0;
        # consts = Constraints()
        # consts.orientation_constraints = [orien_const]
        # right_arm.set_path_constraints(consts)


        # #Set the goal state to the pose you just defined
        # right_arm.set_pose_target(goal_2)


        # #Set the start state for the right arm
        # right_arm.set_start_state_to_current_state()


        # #Plan a path
        # right_plan2 = right_arm.plan()

        # #Execute the plan
        # # raw_input('Press <Enter> to move the right arm to contact.')
        # right_arm.execute(right_plan2)

        # # # # # time.sleep(0.1)



        # Make contact ----------------------------------------------

        right_arm.set_max_velocity_scaling_factor(0.22)

        goal_3 = PoseStamped()
        goal_3.header.frame_id = "ar_marker_4"

        #x, y, and z position
        goal_3.pose.position.y = point3D.y
        goal_3.pose.position.x = point3D.x
        goal_3.pose.position.z = .15
        
        #Orientation as a quaternion
        goal_3.pose.orientation.x = 0.0
        goal_3.pose.orientation.y = -1.0
        goal_3.pose.orientation.z = 0.0
        goal_3.pose.orientation.w = 0.0



        #Set the goal state to the pose you just defined
        right_arm.set_pose_target(goal_3)


        #Set the start state for the right arm
        right_arm.set_start_state_to_current_state()


        #Plan a path
        right_plan3 = right_arm.plan()

        #Execute the plan
        raw_input('Press <Enter> to move the right arm to contact.')
        right_arm.execute(right_plan3)



        # Rotate wrist -------------------------------------------
        raw_input('Press <Enter> to erase point.')
        jpm.main()
    except KeyboardInterrupt:
        print "keyboard interrupt, exiting"




if __name__ == '__main__':
    rospy.init_node('moveit_node')
    while not rospy.is_shutdown():
        rospy.wait_for_service('last_centroids')
        try:
            response = rospy.ServiceProxy('last_centroids', CentroidSrv)
            point = response().centroid
            moveit_erase(point)
        except rospy.ServiceException, e:
            print "Failed to retrieve target point."
            pass 
        except KeyboardInterrupt:
            print "keyboard interrupt, exiting"

    #Start a node
    # rospy.init_node('moveit_node')
    # rospy.Subscriber("/centroids", Point, moveit_erase)
    # rospy.spin()

    # Start a node
    # rospy.init_node('moveit_node')
    # rospy.Subscriber("/centroids", Point, moveit_erase)
    # rospy.spin()