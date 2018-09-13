#!/usr/bin/env python
# -*- coding: utf-8 -*-
import pyproj
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
import csv

MARKERS_MAX =1000000
EPSG4612 = pyproj.Proj("+init=EPSG:4612")
EPSG2451 = pyproj.Proj("+init=EPSG:2451")
pub_msg = PoseWithCovarianceStamped()
sub_msg = PoseWithCovarianceStamped()

def callback(msg):
    #print "\n"
    sub_msg.pose.pose.position.x = msg.pose.pose.position.x
    sub_msg.pose.pose.position.y = msg.pose.pose.position.y
    lat = msg.pose.pose.position.x#msg.latitude
    lon = msg.pose.pose.position.y#msg.longitude
    #print lat, " ", lon
    #lat = 35.679933
    #lon = 139.714465
    
    y,x = pyproj.transform(EPSG4612, EPSG2451, lon,lat)
    #print "transformed to x :", x, "y: ", y
    pub_msg.header = msg.header
    pub_msg.header.frame_id = "/gnss"
    pub_msg.pose = msg.pose
    pub_msg.pose.pose.position.x = x
    pub_msg.pose.pose.position.y = y

def main():
    rospy.init_node('gnss_llh_to_xyz', anonymous=True)

    rospy.Subscriber("latlon", PoseWithCovarianceStamped, callback, queue_size=1)
    pub = rospy.Publisher("latlon_xyz",PoseWithCovarianceStamped,queue_size=1)

    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        pub.publish(pub_msg)
        r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass

