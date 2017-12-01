#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints, CollisionObject
from geometry_msgs.msg import PoseStamped, TransformStamped
import joint_position_manual as jpm
import numpy as np

# def main():
# 	points3D = np.array([])
# 	ARtagslocations = np.array([])
# 	move_and_erase(points3D, ARtagslocations)



def main():
    #Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    #Start a node
    rospy.init_node('moveit_node')

    #Initialize arm
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    right_arm = moveit_commander.MoveGroupCommander('right_arm')
    right_arm.set_planner_id('RRTConnectkConfigDefault')
    right_arm.set_planning_time(10)
  

    #First goal pose ------------------------------------------------------
    goal_1 = PoseStamped()
    goal_1.header.frame_id = "ar_marker_13"

    #x, y, and z position
    goal_1.pose.position.x = 0.0
    goal_1.pose.position.y = 0.0
    goal_1.pose.position.z = 0.1
    
    #Orientation as a quaternion
    goal_1.pose.orientation.x = 0.0
    goal_1.pose.orientation.y = -1.0
    goal_1.pose.orientation.z = 0.0
    goal_1.pose.orientation.w = 0.0

    #Set the goal state to the pose you just defined
    right_arm.set_pose_target(goal_1)

    #Set the start state for the right arm
    right_arm.set_start_state_to_current_state()


    #set workspace
    minX = -1.0
    minY = -1.0
    maxX = 5.0
    maxY = 1.0

    right_arm.set_workspace([minX, minY, maxX, maxY])


    # # Calculate normal vector

    # # tl bl tr br
    # ARtaglocations = np.array([[2.0,-0.2,2.0],
    # [2.0,-0.2,0.0],
    # [2.0,0.2,2.0],
    # [2.0,0.2,0.0]])

    # normalToPlane = np.cross(ARtaglocations[1]-ARtaglocations[2], ARtaglocations[3]-ARtaglocations[2])

    # d = -(normalToPlane[0]*ARtaglocations[0][0]+normalToPlane[1]*ARtaglocations[0][1]+normalToPlane[2]*ARtaglocations[0][2])

    # p = PoseStamped()
    # scene.add_plane("board", p, normal = (normalToPlane[0], normalToPlane[1], normalToPlane[2]), offset = d)

    #Create a path constraint for the arm
    orien_const = OrientationConstraint()
    orien_const.link_name = "right_gripper";
    orien_const.header.frame_id = "ar_marker_13";
    orien_const.orientation.y = -1.0;
    orien_const.absolute_x_axis_tolerance = 0.1;
    orien_const.absolute_y_axis_tolerance = 0.1;
    orien_const.absolute_z_axis_tolerance = 0.1;
    orien_const.weight = 1.0;
    consts = Constraints()
    consts.orientation_constraints = [orien_const]
    right_arm.set_path_constraints(consts)





    # _pose = PoseStamped()

    # scene.add_plane("right_wall", _pose, normal = (0.0, -1.0, 0.0), offset = 0.75)
    # scene.add_plane("left_wall", _pose, normal = (0.0, -1.0, 0.0), offset = -0.75)
    # scene.add_plane("back_wall", _pose, normal = (1.0, 0.0, 0.0), offset = -0.75)





    

    # pos_const = CollisionObject()
    # pos_const.operation = CollisionObject.ADD
    # pos_const.id = "board"
    # pos_const.planes = [[normalToPlane[0], normalToPlane[1], normalToPlane[2]]]
    # scene._pub_co.publish(pos_const)

    # pos_const.meshes[0].vertices[0].x = AR position 1X    
    # pos_const.meshes[0].vertices[0].y = AR position 1y
    # pos_const.meshes[0].vertices[0].z = AR position 1z

    # pos_const.meshes[0].vertices[1].x = AR position 2X    
    # pos_const.meshes[0].vertices[1].y = AR position 2y
    # pos_const.meshes[0].vertices[1].z = AR position 2z

    # pos_const.meshes[0].vertices[2].x = AR position 3X    
    # pos_const.meshes[0].vertices[2].y = AR position 3y
    # pos_const.meshes[0].vertices[2].z = AR position 3z

    # pos_const.meshes[0].vertices[3].x = AR position 4X    
    # pos_const.meshes[0].vertices[3].y = AR position 4y
    # pos_const.meshes[0].vertices[3].z = AR position 4z

    

    # ---------------------- End Constraints -----------------------
    


    #Plan a path
    right_plan = right_arm.plan()

    #Execute the plan
    raw_input('Press <Enter> to move the right arm to goal pose.')
    right_arm.execute(right_plan)

    # raw_input('Press <Enter> to erase point.')
    # jpm.main()


if __name__ == '__main__':
    main()
