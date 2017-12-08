#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints, CollisionObject, MotionPlanRequest
from geometry_msgs.msg import PoseStamped, TransformStamped, Point
import joint_position_manual as jpm
import numpy as np

def main(points3D):
	moveit_erase(points3D)



def moveit_erase(points3D):

    #Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    #Initialize arm
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    right_arm = moveit_commander.MoveGroupCommander('right_arm')

    right_arm.set_planner_id('RRTConnectkConfigDefault')
    right_arm.set_planning_time(15) # TEST time length

    # Set plane as a collision object in workspace ------------------FINISH/TEST CODE----------------

    # def callback(message):
        # return tag info needed to find z-axis in base frame and find full plane equation (ie normal vector and a point on the plane)

    # ARtaglocations = rospy.Subscriber("/tf", ----DataType-----, callback)

    # normalToPlane = ARtaglocations[0](z-axis in base frame)

    # d = -(normalToPlane[0]*ARtaglocations[0][0]+normalToPlane[1]*ARtaglocations[0][1]+normalToPlane[2]*ARtaglocations[0][2])

    # p = PoseStamped()
    # scene.add_plane("board", p, normal = (normalToPlane[0], normalToPlane[1], normalToPlane[2]), offset = d)

    #Set the velocity of the joint movement
    right_arm.set_max_velocity_scaling_factor(0.3)

    #Set joint targets for start position ------------TEST CODE----------if works can use for wrist twisting-----------
    start_pos = {
        'right_j0': 0.111921875,
        'right_j1': 1.24400390625,
        'right_j2': -2.936244140625,
        'right_j3': 2.8404443359375,
        'right_j4': 0.19019042967875,
        'right_j5': -2.361208007815,
        'right_j6': 0.1680185546875
        }

    #Set start start_pos as joint target
    right_arm.set_joint_value_target(start_pos)

    p = PoseStamped()
    scene.add_plane("left", p, normal = (0.0, 1.0, 0.0), offset = -0.6)    
    scene.add_plane("right", p, normal = (0.0, -1.0, 0.0), offset = -0.6)    
    scene.add_plane("wall", p, normal = (-1.0, 0.0, 0.0), offset = -0.9)


    #Plan a path
    right_plan0 = right_arm.plan()

    #Execute the plan
    raw_input('Press <Enter> to move the right arm to start pose.')
    right_arm.execute(right_plan0)


    #Move towards board ------------------------------------------------------
    # print(points3D)
    # goal_1 = PoseStamped()
    # goal_1.header.frame_id = "ar_marker_4"

    # #x, y, and z position
    # goal_1.pose.position.x = points3D.x
    # goal_1.pose.position.y = points3D.y
    # goal_1.pose.position.z = 0.185

    # #Orientation as a quaternion
    # goal_1.pose.orientation.x = 0.0
    # goal_1.pose.orientation.y = -1.0
    # goal_1.pose.orientation.z = 0.0
    # goal_1.pose.orientation.w = 0.0


    # #Set the goal state to the pose you just defined
    # right_arm.set_pose_target(goal_1)

    # minX = -0.9
    # minY = -0.6
    # maxX = 5.0
    # maxY = 0.6
    # right_arm.set_workspace([minX, minY, maxX, maxY])

    # #Set the start state for the right arm
    # right_arm.set_start_state_to_current_state()




    # #Plan a path
    # right_plan = right_arm.plan()

    # #Execute the plan
    # raw_input('Press <Enter> to move the right arm to goal pose.')
    # right_arm.execute(right_plan)


    #Erase board -------------------------TEST CODE-----------------------------

    posPi = {
    'right_j6': 3.14
    }

    posWristNeutral = {
    'right_j6': 0
    }
    

    #Set start start_pos as joint target
    right_arm.set_joint_value_target(posWristNeutral)

    #Plan a path
    right_plan3 = right_arm.plan()

    #Execute the plan
    right_arm.execute(right_plan3)


    #Set start start_pos as joint target
    right_arm.set_joint_value_target(posPi)

    #Plan a path
    right_plan4 = right_arm.plan()

    #Execute the plan
    right_arm.execute(right_plan4)


    #Set start start_pos as joint target
    right_arm.set_joint_value_target(posWristNeutral)

    #Plan a path
    right_plan5 = right_arm.plan()

    #Execute the plan
    right_arm.execute(right_plan5)




    # # RUN SEPARATELY TO SEE IF COMMENTED OUT CODE IS NECESSARY ----------TEST CODE----------------
    # raw_input('Press <Enter> to erase point.')
    # jpm.main()




if __name__ == '__main__':

    #Start a node
    rospy.init_node('moveit_node')
    rospy.Subscriber("/centroids", Point, main)
    rospy.spin()

