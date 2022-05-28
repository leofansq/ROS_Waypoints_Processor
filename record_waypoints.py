#!/usr/bin/env python
# coding:utf-8
import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

from math import sqrt

def callback(msg):
    global last_point, waypoint_filename, waypoint_interval, marker_pub, idx
    
    distance = sqrt((last_point[0]-msg.pose.position.x)**2 + \
               (last_point[1]-msg.pose.position.y)**2 + \
               (last_point[2]-msg.pose.position.z)**2)

    if distance >= waypoint_interval:

        last_point = [msg.pose.position.x, \
                      msg.pose.position.y, \
                      msg.pose.position.z, \
                      msg.pose.orientation.x, \
                      msg.pose.orientation.y, \
                      msg.pose.orientation.z, \
                      msg.pose.orientation.w]
        
        # Record the waypoint
        record_file = open(waypoint_filename, 'a+')
        wp = ",".join(str(i) for i in last_point)
        record_file.write(wp+"\n")
        record_file.close()

        # Marker generation
        marker = Marker()

        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.id = idx
        idx += 1
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = msg.pose.position.x
        marker.pose.position.y = msg.pose.position.y
        marker.pose.position.z = msg.pose.position.z
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker_pub.publish(marker)
    

def waypoint_recorder(sub="/ndt_pose", file_name='./waypoints.txt', interval=1):
    """
    Parameters:
        sub: the geometry_msgs::PoseStamped topic to subcribe
        file_name: the name of the waypoints record
        interval: the distance between two waypoints (m)
    """
    global last_point, waypoint_filename, waypoint_interval, marker_pub, idx

    last_point = [0, 0, 0, 0, 0, 0, 0]
    waypoint_filename = file_name
    waypoint_interval = interval
    idx = 0

    rospy.init_node('waypoint_recorder', anonymous=True)
    rospy.loginfo("Ready to record the waypoints......")

    marker_pub= rospy.Publisher('/waypoint_marker', Marker, queue_size=10);  
    rospy.Subscriber(sub, PoseStamped, callback)

    rospy.spin()

if __name__=='__main__':
    waypoint_recorder(sub="/ndt_pose", file_name='./waypoints.txt', interval=1)

