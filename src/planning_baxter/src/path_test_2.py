#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
import pointTransFunctions as ptf
import numpy as np
from ar_track_alvar_msgs import AlvarMarkers

tag_poses = []

def callback(message):
    markers = msg.markers
    for marker in markers:
        pose = marker.pose.pose
        coords = np.array([pose.position.x, pose.position.y, pose.position.z])
        orientation = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        tag_poses.append((coords, orientation))

def listener():
    rospy.init.node('ar_pose_reader')
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback)


def main(pointList):
    listener()

    #Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    #Start a node
    rospy.init_node('moveit_node')

    #Initialize both arms
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
#    left_arm = moveit_commander.MoveGroupCommander('left_arm')
    right_arm = moveit_commander.MoveGroupCommander('right_arm')
#    left_arm.set_planner_id('RRTConnectkConfigDefault')
#    left_arm.set_planning_time(10)
    right_arm.set_planner_id('RRTConnectkConfigDefault')
    right_arm.set_planning_time(10)

    #Retrieve 3D tag points from /ar_pose_marker: tl, bl, tr, br
    
    point1 = np.array([tag_poses[0][0][0], tag_poses[0][0][1], tag_poses[0][0][2]])
    point2 = np.array([tag_poses[1][0][0], tag_poses[1][0][1], tag_poses[1][0][2]])

    if point1[0] < point2[0]:
        tagPoints3DAR = np.array([[point1], [point2]])
    else:
        tagPoints3DAR = np.array([[point2], [point1]])

    #Calculate other two points: bl, tr


    X_LBOUND = 0
    X_UBOUND = 0
    Y_LBOUND = 0
    Y_UBOUND = 0
    Z_LBOUND = 0
    Z_UBOUND = 0

    #Retrieve pixel tag points: tl, bl, tr, br
    tagPointsPixel = np.array([[],
        [],
        [],
        []])

    #Calculate height and length of board
    hboard = np.linalg.norm(tagPoints3D[0]-tagPoints3D[1])
    lboard = np.linalg.norm(tagPoints3D[3]-tagPoints3D[1]) 

    #Calculate H transform
    Htrans = ptf.pixelTo2DPlaneTransform(hboard, lboard, tagPointsPixel)

    for p in pointList:

        point2D = ptf.pixelTo2DPoint(p, Htrans)

        point3D = ptf.PlaneTo3DPoint(point2D[0], point2D[1], ...
            hboard, lboard, tagPoints3D)

        if point3D[1] < Y_LBOUND or point3D[1] > Y_UBOUND:
        	continue
        if point3D[0] < X_LBOUND or point3D[0] > X_UBOUND:
        	continue
        if point3D[2] < Z_LBOUND or point3D[2] > Z_UBOUND:
        	continue


        rospy.sleep(2.0)
        goal = PoseStamped()
        goal.header.frame_id = "base"

        #x, y, and z position
        goal.pose.position.x = point3D[0]
        goal.pose.position.y = point3D[1]
        goal.pose.position.z = point3D[2]
    
        #Orientation as a quaternion
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = -1.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 0.0

        #Set the goal state to the pose you just defined
        right_arm.set_pose_target(goal)

        #Set the start state for the right arm
        right_arm.set_start_state_to_current_state()

        # #Create a path constraint for the arm
        # #UNCOMMENT TO ENABLE ORIENTATION CONSTRAINTS
        # orien_const = OrientationConstraint()
        # orien_const.link_name = "right_gripper";
        # orien_const.header.frame_id = "base";
        # orien_const.orientation.y = -1.0;
        # orien_const.absolute_x_axis_tolerance = 0.1;
        # orien_const.absolute_y_axis_tolerance = 0.1;
        # orien_const.absolute_z_axis_tolerance = 0.1;
        # orien_const.weight = 1.0;
        # consts = Constraints()
        # consts.orientation_constraints = [orien_const]
        # right_arm.set_path_constraints(consts)

        #Plan a path
        right_plan = right_arm.plan()

        #Execute the plan
        right_arm.execute(right_plan)


if __name__ == '__main__':
    main()
