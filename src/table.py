#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, tf2_ros

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, TransformStamped
from std_msgs.msg import Header, ColorRGBA
from tf.transformations import quaternion_from_euler

if __name__=='__main__':
    rospy.init_node('table')            #Initialisation du node 'table'
    rospy.sleep(1)                      
    
    br = tf2_ros.TransformBroadcaster() #Définit l'emetteur de données
    t = TransformStamped()              #Permet la transformation du parent à l'enfant    
    fear = TransformStamped()
    
    x_table, y_table, z_table = 0, 0.50, 0.65   #Diverses variables pour le marker table
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
    fear.transform.translation.y = 0.25
    fear.transform.translation.z = 0.04
    fear.transform.rotation.x = -0.500
    fear.transform.rotation.y = 0.497
    fear.transform.rotation.z = -0.486
    fear.transform.rotation.w = 0.516
    
    marker_publisher = rospy.Publisher('visualization_marker',       #Publication topic visualization_marker 
                                        Marker, 
                                        queue_size=5)           
    table = Marker(type=Marker.CUBE,                                        #Forme du marker    #Définition du marker
                    color=ColorRGBA(255,255,255,0.8),                       #Couleur
                    pose=Pose(Point(x_table, y_table, z_table),             #Position
                    Quaternion(qx_table, qy_table, qz_table, qw_table)),    #Rotation
                    scale=Vector3(0.01, 1.50, 1.50),                        #Taille
                    header=Header(frame_id='world'))                        #Parent
    
    while not rospy.is_shutdown():              #Tant que le processus n'est pas quitté
        marker_publisher.publish(table)         #Publication du marker table
        t.header.stamp = rospy.Time.now()       #Définit le temps de l'entête
        br.sendTransform(t)                     #Envoie la transformation de l'enfant au parent sur le topic ROS
        fear.header.stamp = rospy.Time.now()
        br.sendTransform(fear)
        print("Marker published!")
        rospy.sleep(2)

