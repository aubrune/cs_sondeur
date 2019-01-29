#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, tf2_ros
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, TransformStamped
from std_msgs.msg import Header, ColorRGBA
from tf.transformations import quaternion_from_euler

if __name__=='__main__':
    rospy.init_node('sondeur')
    rospy.sleep(1)
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    quartenion = quaternion_from_euler(0, 0.05, 0)
    
    x_table, y_table, z_table = 0.50, 0, 0.75
    qx_table, qy_table, qz_table, qw_table = quartenion[0], quartenion[1], quartenion[2], quartenion[3]
    t.header.frame_id = "world"
    t.child_frame_id = "table"
    t.transform.translation.x = x_table
    t.transform.translation.y = y_table
    t.transform.translation.z = z_table
    t.transform.rotation.x = qx_table
    t.transform.rotation.y = qy_table
    t.transform.rotation.z = qz_table
    t.transform.rotation.w = qw_table

    
    marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)
    table = Marker(type=Marker.CUBE, color=ColorRGBA(255,255,255,0.8), pose=Pose(Point(x_table, y_table, z_table), Quaternion(qx_table, qy_table, qz_table, qw_table)), scale=Vector3(0.01, 1.50, 1.50), header=Header(frame_id='world'))
    while not rospy.is_shutdown():
        marker_publisher.publish(table)
        t.header.stamp = rospy.Time.now()
        br.sendTransform(t)
        print("Marker published!")
        rospy.sleep(1)
