#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
import pointTransFunctions as ptf
import numpy as np


def main(pointList):
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
    tagPoints3D = np.array([[],
        [],
        [],
        []])

    #Retrieve pixel tag points: tl, bl, tr, br
    tagPointsPixel = np.array([[],
        [],
        [],
        []])

    #Calculate height and length of board
    hboard = np.linalg.norm(tagPoints3D[0]-tagPoints3D[1])
    lboard = np.linalg.norm(tagPoints3D[3]-tagPoints3D[1]) 

    #Calculate H transform
    Htrans = pts.pixelTo2DPlaneTransform(hboard, lboard, tagPointsPixel)

    for p in pointList:

        point2D = pts.pixelTo2DPoint(p, Htrans)

        point3D = pts.PlaneTo3DPoint(point2D[0], point2D[1], ...
            hboard, lboard, tagPoints3D)

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
