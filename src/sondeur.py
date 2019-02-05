#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, tf2_ros
from visualization_msgs.msg import Marker
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, TransformStamped
from std_msgs.msg import Header, ColorRGBA
from tf.transformations import quaternion_from_euler
import moveit_commander
import moveit_msgs.msg
from math import pi
import sys

import copy

if __name__=='__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('sondeur')          #Initialisation du node 'sondeur'
    rospy.sleep(1)                      
    
    br = tf2_ros.TransformBroadcaster() #Définit l'emetteur de données
    t = TransformStamped()              #Permet la transformation du parent à l'enfant    
    fear = TransformStamped()
    
    x_table, y_table, z_table = 0, 0.50, 0.75   #Diverses variables pour le marker table
    quartenion = quaternion_from_euler(0, 0, 1.5707)
    qx_table, qy_table, qz_table, qw_table = quartenion[0], quartenion[1], quartenion[2], quartenion[3]
    
    t.header.frame_id = "world"                 #Repère tableau
    t.child_frame_id = "table"
    t.transform.translation.x = x_table
    t.transform.translation.y = y_table
    t.transform.translation.z = z_table
    t.transform.rotation.x = qx_table
    t.transform.rotation.y = qy_table
    t.transform.rotation.z = qz_table
    t.transform.rotation.w = qw_table
    
    fear.header.frame_id = "table"              #Repère début de la colonne peur
    fear.child_frame_id = "fear_start"
    fear.transform.translation.x = 0
    fear.transform.translation.y = 0.5
    fear.transform.translation.z = 0.10
    fear.transform.rotation.x = 0
    fear.transform.rotation.y = 0
    fear.transform.rotation.z = 0
    fear.transform.rotation.w = 1
    
    marker_publisher = rospy.Publisher('visualization_marker',       #Publication topic visualization_marker 
                                        Marker, 
                                        queue_size=5)           
    table = Marker(type=Marker.CUBE,                                        #Forme du marker    #Définition du marker
                    color=ColorRGBA(255,255,255,0.8),                       #Couleur
                    pose=Pose(Point(x_table, y_table, z_table),             #Position
                    Quaternion(qx_table, qy_table, qz_table, qw_table)),    #Rotation
                    scale=Vector3(0.01, 1.50, 1.50),                        #Taille
                    header=Header(frame_id='world'))                        #Parent
    
    
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    
    group_name = "edo"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    #panner_id = moveit_commander.MoveGroupCommander.set_planner_id("RRTConnectkConfigDefault")
    # We can get the joint values from the group and adjust some of the values:
    #joint_goal = move_group.get_current_joint_values()
    #joint_goal[0] = 0
    #joint_goal[1] = -pi/4
    #joint_goal[2] = 0
    #joint_goal[3] = -pi/2
    #joint_goal[4] = 0
    #joint_goal[5] = pi/3

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    #move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    #move_group.stop()
    move_group.set_planner_id("RRTConnectkConfigDefault")
    
    move_group.set_planning_time(10)
    print move_group.get_current_pose()
    pose_goal = geometry_msgs.msg.Pose()

    pose_goal.position.x = -0.076
    pose_goal.position.y = 0.366
    pose_goal.position.z = 0.294
    pose_goal.orientation.x = -0.470
    pose_goal.orientation.y = 0.493
    pose_goal.orientation.z = 0.547
    pose_goal.orientation.w = -0.486
    
    move_group.set_pose_target(pose_goal)
    #plan = move_group.go(wait=True)
    move_group.go(wait = True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    #move_group.clear_pose_targets()
       
    
    while not rospy.is_shutdown():              #Tant que le processus n'est pas quitté
        marker_publisher.publish(table)         #Publication du marker table
        t.header.stamp = rospy.Time.now()       #Définit le temps de l'entête
        br.sendTransform(t)                     #Envoie la transformation de l'enfant au parent sur le topic ROS
        fear.header.stamp = rospy.Time.now()
        br.sendTransform(fear)
        print("Marker published!")
        rospy.sleep(1)

