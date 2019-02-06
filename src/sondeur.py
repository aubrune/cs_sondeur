#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, tf2_ros
import moveit_commander
import sys

from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

if __name__=='__main__':
    rospy.init_node('sondeur')          #Initialisation du node 'sondeur'
    rospy.sleep(1)                      
    
    group_name = "edo"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group.set_planner_id("RRTConnectkConfigDefault")   
    move_group.set_planning_time(10)
    
    print move_group.get_current_pose()
    
    pose_goal = Pose()
    
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(10.0)
    
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('world', 'fear_start', rospy.Time())
            print("Possible")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Erreur")
            rate.sleep()
            continue
        
        pose_goal.position.x = trans.transform.translation.x
        pose_goal.position.y = trans.transform.translation.y - 0.10
        pose_goal.position.z = trans.transform.translation.z
        pose_goal.orientation.x = 0.099
        pose_goal.orientation.y = 0.665
        pose_goal.orientation.z = 0.733
        pose_goal.orientation.w = -0.106
        
        move_group.set_pose_target(pose_goal)
        move_group.go(wait = True)
        move_group.stop()
        move_group.clear_pose_targets() 
        break   
          
        
      
