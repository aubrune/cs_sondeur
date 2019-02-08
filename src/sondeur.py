#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, tf2_ros
import moveit_commander
import sys

from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

nb_votes = 0
nb_blocks = 0
nb_sticks = 0

marker_gap = 0.03
sticks_gap = 0.015
stick_length = 0.04

def go_to_initial_position():
    print("En attente de transformation")
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('world', 'table', rospy.Time())
            print("Transformation possible")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        
        print("Déplacement au marqueur fear_start")
        pose_goal.position.x = trans.transform.translation.x
        pose_goal.position.y = trans.transform.translation.y - 0.10 + 0.03
        pose_goal.position.z = trans.transform.translation.z
        
        quartenion = quaternion_from_euler(-1.537, 0.001, -0.001)
        qx_robot, qy_robot, qz_robot, qw_robot = quartenion[0], quartenion[1], quartenion[2], quartenion[3]
        
        pose_goal.orientation.x = qx_robot
        pose_goal.orientation.y = qy_robot
        pose_goal.orientation.z = qz_robot
        pose_goal.orientation.w = qw_robot
        
        move_group.set_pose_target(pose_goal)
        move_group.go(wait = True)
        move_group.stop()
        move_group.clear_pose_targets() 
        break   

def trace_stick():
    #Fonction traçant un baton pour les 4 premiers votes de chaque bloc
    move_forward()
    print "Trace un trait"
    pose_goal.position.z -= stick_length
    move_group.set_pose_target(pose_goal)
    move_group.go(wait = True)
    move_group.stop()
    move_group.clear_pose_targets() 
    move_back()

def trace_line():
    #Fonction traçant un trait en diagonale pour le 5 vote de chaque bloc
    move_forward()
    print "Trace une ligne"
    pose_goal.position.z -= stick_length
    pose_goal.position.x -= 3 * sticks_gap
    move_group.set_pose_target(pose_goal)
    move_group.go(wait = True)
    move_group.stop()
    move_group.clear_pose_targets()
    move_back()
    
def move_back():
    print "Recule"
    pose_goal.position.y -= marker_gap
    move_group.set_pose_target(pose_goal)
    move_group.go(wait = True)
    move_group.stop()
    move_group.clear_pose_targets() 

def move_forward():
    print "Avance"
    pose_goal.position.y += marker_gap
    move_group.set_pose_target(pose_goal)
    move_group.go(wait = True)
    move_group.stop()
    move_group.clear_pose_targets() 

def move_to_next_stick():
    print "Passe au prochain trait"
    pose_goal.position.z += stick_length
    pose_goal.position.x += sticks_gap
    move_group.set_pose_target(pose_goal)
    move_group.go(wait = True)
    move_group.stop()
    move_group.clear_pose_targets() 

if __name__=='__main__':
    rospy.init_node('sondeur')          #Initialisation du node 'sondeur'
    rospy.sleep(1)                      
    
    group_name = "edo"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group.set_planner_id("RRTConnectkConfigDefault")   
    move_group.set_planning_time(10)
    
    pose_goal = Pose()
    
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(10.0)
    go_to_initial_position()
    while not rospy.is_shutdown():
        vote = raw_input("Appuyez sur une touche puis sur entrée pour voter : ")
        nb_votes += 1
        print "Vote n°", nb_votes
        print "Résultat du modulo 5 :", nb_votes % 5
        if nb_votes % 5 == 0:
            trace_line()
        else:
            if nb_votes != 1:
                move_to_next_stick()
            trace_stick()

