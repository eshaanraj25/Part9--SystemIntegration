#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from styx_msgs.msg import Lane, Waypoint

import numpy as np
from scipy.spatial import KDTree

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

# Constants
LOOKAHEAD_WPS = 50 
MAX_DECEL = 0.5
STOPLINE_OFFSET = 4
CONSTANT_DECEL = 1 / LOOKAHEAD_WPS

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Initialize class variables
        self.base_lane = None
        self.pose = None
        self.stopline_waypoint_index = -1
        self.waypoints_2d = None
        self.waypoint_tree = None

        self.loop()

    def loop(self):
        # ROSPy loop to publish waypoints to follow
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_lane and self.waypoint_tree:
                self.publish_waypoints()
            rate.sleep()

    def get_closest_waypoint_index(self):
        # Function to get index of closest waypoint to given pose
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_index = self.waypoint_tree.query([x, y], 1)[1]

        # Check if closest is ahead or behind the vehicle
        closest_coordinates = self.waypoints_2d[closest_index]
        previous_coordinates = self.waypoints_2d[closest_index - 1]

        # Dot product to determine the point ahead of car
        close_vector = np.array(closest_coordinates)
        previous_vector = np.array(previous_coordinates)
        position_vector = np.array([x, y])

        product = np.dot(close_vector - previous_vector, position_vector - close_vector)

        if product > 0:
            closest_index = (closest_index + 1) % len(self.waypoints_2d)

        return closest_index

    def publish_waypoints(self):
        # Function to generate a Lane Message and publish it
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):
        # Function to generate a Lane Message
        lane = Lane()

        # Generate boundary indices
        closest_index = self.get_closest_waypoint_index()
        farthest_index = closest_index + LOOKAHEAD_WPS
        base_waypoints = self.base_lane.waypoints[closest_index : farthest_index]

        # Conditions based on boundary indices
        if self.stopline_waypoint_index == -1 or self.stopline_waypoint_index >= farthest_index:
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_index)

        return lane
    
    def decelerate_waypoints(self, waypoints, closest_index):
        # Function to calculate waypoints while decelerating
        result = []

        # Iterate through the waypoints
        for index, waypoint in enumerate(waypoints):
            point = Waypoint()
            point.pose = waypoint.pose

            # Adjust velocity to slow down
            stop_index = max(self.stopline_waypoint_index - closest_index - STOPLINE_OFFSET, 0)
            distance = self.distance(waypoints, index, stop_index)
            velocity = math.sqrt(2 * MAX_DECEL * distance) + (index * CONSTANT_DECEL)
            if velocity < 1:
                velocity = 0

            point.twist.twist.linear.x = min(velocity, waypoint.twist.twist.linear.x)
            result.append(point)

        return result

    def pose_cb(self, msg):
        # Call back function for current pose
        self.pose = msg

    def waypoints_cb(self, waypoints):
        # Call back function for base waypoints
        self.base_lane = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] 
                                for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # Call back function for Traffic Lights
        self.stopline_waypoint_index = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
