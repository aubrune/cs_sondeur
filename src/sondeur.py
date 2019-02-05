#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, tf2_ros
import moveit_commander
import sys

from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

if __name__=='__main__':
    #moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('sondeur')          #Initialisation du node 'sondeur'
    rospy.sleep(1)                      
    
    #robot = moveit_commander.RobotCommander()
    #scene = moveit_commander.PlanningSceneInterface()
    
    group_name = "edo"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group.set_planner_id("RRTConnectkConfigDefault")
    
    move_group.set_planning_time(10)
    print move_group.get_current_pose()
    pose_goal = Pose()
    
    #quartenion_goal = quaternion_from_euler(0, 0, 1.5707)

    pose_goal.position.x = -0.076
    pose_goal.position.y = 0.366
    pose_goal.position.z = 0.294
    pose_goal.orientation.x = -0.470
    pose_goal.orientation.y = 0.493
    pose_goal.orientation.z = 0.547
    pose_goal.orientation.w = -0.486
    #pose_goal.orientation.normalize();
    
    move_group.set_pose_target(pose_goal)
    #plan = move_group.go(wait=True)
    move_group.go(wait = True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()
      
