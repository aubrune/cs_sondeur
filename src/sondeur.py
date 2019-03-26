#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, tf2_ros
import moveit_commander
import copy

from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String

MARKER_GAP = 0.03       #Espacement entre le marqueur et le tableau
STICKS_GAP = 0.015      #Espacement entre les batons de vote
STICK_LENGTH = 0.04     #Longueur du trait du baton
BLOCKS_GAP = 0.03       #Espacement entre les blocs
LINE_GAP = 0.03         #Espacement entre les lignes de vote

NB_MAX_BLOCKS = 2       #Nombre maximum de blocs sur chaque ligne

nb_total_votes = 0      #Nombre total de votes
nb_total_blocks = 1     #Nombre total de blocs
nb_votes_block = 0      #Nombre de votes à l'intérieur du bloc courant

def go_to_initial_position():   #Fonction qui amène le robot à la position de départ
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
        print pose_goal.position.x
        pose_goal.position.y = trans.transform.translation.y - 0.10 + 0.03
        pose_goal.position.z = trans.transform.translation.z
        
        quartenion = quaternion_from_euler(-1.537, 0.001, -0.001)
        qx_robot, qy_robot, qz_robot, qw_robot = quartenion[0], quartenion[1], quartenion[2], quartenion[3]
        
        pose_goal.orientation.x = qx_robot
        pose_goal.orientation.y = qy_robot
        pose_goal.orientation.z = qz_robot
        pose_goal.orientation.w = qw_robot
        
        go_to_pose_goal()
        break   

def trace_stick():              #Fonction traçant un baton
    move_forward()
    print "Trace un trait"
    #pose_goal.position.z -= STICK_LENGTH
    #go_to_pose_goal()
    waypoints = []
    wpose = move_group.get_current_pose().pose
    wpose.position.z -= STICK_LENGTH
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold
    move_group.execute(plan, wait=True)
    move_back()

def trace_line():               #Fonction traçant un trait en diagonale
    move_up()
    move_forward()
    print "Trace une ligne"
    pose_goal.position.z -= STICK_LENGTH
    pose_goal.position.x -= 3 * STICKS_GAP
    go_to_pose_goal()
    move_back()
    
def move_up():                  #Fonction qui remonte le bras pour le préparer à tracer une ligne en diagonale
    print "Remonte"
    pose_goal.position.z += STICK_LENGTH
    go_to_pose_goal()
    
def move_back():                #Fonction qui recule le marqueur du tableau
    print "Recule"
    pose_goal.position.y -= MARKER_GAP
    go_to_pose_goal()

def move_forward():             #Fonction qui avance le marqueur vers le tableau
    print "Avance"
    pose_goal.position.y += MARKER_GAP
    go_to_pose_goal()

def move_to_next_stick():       #Fonction qui bouge le marqueur au prochain baton
    print "Passe au prochain trait"
    pose_goal.position.z += STICK_LENGTH
    pose_goal.position.x += STICKS_GAP
    go_to_pose_goal()
    
def go_to_pose_goal():          #Fonction qui valide la trajectoire du bras
    move_group.set_pose_target(pose_goal)
    move_group.go(wait = True)
    move_group.stop()
    move_group.clear_pose_targets() 

def change_line():             #Fonction qui change la ligne courante
    print "Change de ligne"
    pose_goal.position.z -= LINE_GAP
    pose_goal.position.x -= (3 * STICKS_GAP) * NB_MAX_BLOCKS + (NB_MAX_BLOCKS - 1) * BLOCKS_GAP
    test = (3 * STICKS_GAP) * NB_MAX_BLOCKS + (NB_MAX_BLOCKS - 1) * BLOCKS_GAP
    print pose_goal.position.x
    print test
    go_to_pose_goal()
    
def change_block():             #Fonction qui change le bloc courant
    print "Change de bloc"
    pose_goal.position.x += 3 * STICKS_GAP + BLOCKS_GAP
    pose_goal.position.z += STICK_LENGTH
    go_to_pose_goal()
    
def callback(data):
    global nb_total_votes
    rospy.loginfo(rospy.get_caller_id() + " - Received from topic value : %s", data.data)
    nb_total_votes = int(data.data)
    move_robot()
    
def move_robot():
    global nb_total_votes
    global nb_votes_block
    global nb_total_blocks
    print "Nombre de votes total :", nb_total_votes
    print "Nombre de votes dans le bloc courant :", nb_votes_block
    print "Nombre de blocs :", nb_total_blocks
    #nb_total_votes += 1
    nb_votes_block += 1

    if ((nb_total_votes - 1) % 5) == 0 and nb_total_votes != 1:
        if nb_total_blocks % NB_MAX_BLOCKS == 0:
            change_line()
        else:
            change_block()
            nb_total_blocks += 1
        trace_stick()
    elif nb_total_votes % 5 == 0:
        trace_line()
        nb_votes_block = 0
    else:
        if nb_votes_block != 1:
            move_to_next_stick()
        trace_stick()

if __name__=='__main__':    #MAIN     
    rospy.init_node('edo_movement') #Initialisation du node 'edo_movement'
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
        rospy.Subscriber("vote", String, callback)
        #move_robot()
        rospy.spin()
        
 
