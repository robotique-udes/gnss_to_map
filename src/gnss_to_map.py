#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from geographiclib.geodesic import Geodesic
from sensor_msgs.msg import NavSatFix
from math import radians, cos, sin

def get_origin_lat_long():
  # Get the lat long coordinates of our map frame's origin which must be published on topic /local_xy_origin. We use this to calculate our goal within the map frame.
  rospy.loginfo("Waiting for a message to initialize the origin GPS location...")
  origin_pose = rospy.wait_for_message('local_xy_origin', PoseStamped)
  origin_lat = origin_pose.pose.position.y
  origin_long = origin_pose.pose.position.x
  rospy.loginfo('Received origin: lat %s, long %s.' % (origin_lat, origin_long))
  return origin_lat, origin_long


def calc_geodesic(lat1, long1, lat2, long2):
  # Calculate distance and azimuth between GPS points
  geod = Geodesic.WGS84  # define the WGS84 ellipsoid
  return geod.Inverse(lat1, long1, lat2, long2) # Compute several geodesic calculations between two GPS points 


def calc_map_coord(origin_lat, origin_long, curr_lat, curr_long):
  g = calc_geodesic(origin_lat, origin_long, curr_lat, curr_long)
  hypotenuse = g['s12'] # access distance
  azimuth = g['azi1']

  # Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) by finding side lengths of a right-angle triangle
  # Convert azimuth to radians
  azimuth = radians(azimuth)
  y = cos(azimuth) * hypotenuse
  x = sin(azimuth) * hypotenuse

  return x, y


class GnssToMap:
    def __init__(self):
        rospy.init_node('gnss_to_map', anonymous=True)
        self.origin_lat, self.origin_long = get_origin_lat_long()
        self.map_coord_pub = rospy.Publisher("gnss_map_pose", PoseWithCovarianceStamped, queue_size=1)
        rospy.Subscriber('fix', NavSatFix, self.fixCB)

    def fixCB(self, msg):
        x, y = calc_map_coord(self.origin_lat, self.origin_long, msg.latitude, msg.longitude)
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        pose.pose.pose.position.z = 0
        pose.pose.pose.orientation.x = 0
        pose.pose.pose.orientation.y = 0
        pose.pose.pose.orientation.z = 0
        pose.pose.pose.orientation.w = 1
        position_covariance = np.reshape(msg.position_covariance, (3, 3))
        covariance = np.identity(6, dtype=float)
        covariance[0:3, 0:3] = position_covariance
        pose.pose.covariance = covariance.reshape(36).tolist()

        self.map_coord_pub.publish(pose)

if __name__ == '__main__':
    try:
        gnss_to_map = GnssToMap()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass