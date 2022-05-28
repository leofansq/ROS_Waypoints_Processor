#!/usr/bin/env python
# coding:utf-8

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion


def load_waypoints(waypoints_path):

    # Read the waypoints_file
    waypoints = []
    waypoints_file = open(waypoints_path)
    for line in waypoints_file.readlines():
        waypoints.append([float(i) for i in line.split(',')])
    waypoints_file.close()

    # Generate the waypoints_info
    waypoints_info = Path()
    waypoints_info.header.frame_id = 'map'
    waypoints_info.header.stamp = rospy.Time.now()

    for idx in range(len(waypoints)-1):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position = Point(waypoints[idx][0], waypoints[idx][1], waypoints[idx][2])
        pose.pose.orientation = Quaternion(waypoints[idx][3], waypoints[idx][4], waypoints[idx][5], waypoints[idx][6])
        
        waypoints_info.poses.append(pose)
    
    rospy.loginfo('Fetched ' + str(len(waypoints)) + ' waypoints')

    if waypoints == []:
        rospy.signal_shutdown('No waypoint to draw... Shutdown')

    return waypoints_info


def main():

    rospy.init_node('waypoints_loader')
    waypoint_pub = rospy.Publisher('/waypoints', Path, queue_size=1)
    rate = rospy.Rate(1)

    waypoints_path = rospy.get_param("load_waypoints/waypoints_path")
    waypoints_info = load_waypoints(waypoints_path)

    while not rospy.is_shutdown():
        waypoint_pub.publish(waypoints_info)
        rate.sleep()


if __name__ == "__main__":
    try:    
        rospy.init_node('waypoints_loader')
        waypoint_pub = rospy.Publisher('/waypoints', Path, queue_size=1)
        rate = rospy.Rate(1)

        waypoints_path = "./waypoints_files/waypoints.txt"
        waypoints_info = load_waypoints(waypoints_path)

        while not rospy.is_shutdown():
            waypoint_pub.publish(waypoints_info)
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.logerr('Get KeyBoardInterrupt... Shutdown')