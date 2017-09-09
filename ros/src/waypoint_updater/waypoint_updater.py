#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math, sys
from itertools import islice, cycle

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

===============================

Implemention Strategy (Partial Waypoint Updater)

When you receive a message on the 'base_waypoints' topic, save this to a member var. This will not change once initialized. Future
messages are ignored.

For every message on the 'current_pose' topic:
1. Find the closest waypoint to the vehicle's current position
2. Starting from this waypoint generate a list of the next LOOKAHEAD_WPS points by sequentially iterating through the list starting
   from the closest waypoint.
3. Publish this list to the final_waypoints publisher

'''

LOOKAHEAD_WPS = 10 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.waypoints = None
        self.current_pose = None

        rospy.spin()

    # Callback for the position updater topic
    def pose_cb(self, msg):
        self.current_pose = msg.pose
        rospy.loginfo("Car position updated to %s", self.current_pose);
        self.send_next_waypoints()

    # Callback for the waypoints updater topic
    def waypoints_cb(self, lane):
        # Looks like the waypoints won't change after they're initially sent to us. So once we get the data, we can ignore it later on
        # https://carnd.slack.com/archives/C6NVDVAQ3/p1504803165000409?thread_ts=1504718014.000557&cid=C6NVDVAQ3
        if self.waypoints is None:
            self.waypoints = lane.waypoints
            self.send_next_waypoints()
        
        
    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

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

    def send_next_waypoints(self):
        if self.waypoints is None or self.current_pose is None:
            return

        carx = self.current_pose.position.x
        cary = self.current_pose.position.y

        rospy.loginfo("Finding closest waypoint to car at position %f, %f", carx, cary)
        # find the closest waypoint to the car
        min_dist = sys.maxsize
        min_loc = None
        for i, waypoint in enumerate(self.waypoints):
            wp_x = waypoint.pose.pose.position.x
            wp_y = waypoint.pose.pose.position.y
            # distance calulated with distance formula... sqrt((x1 - x2)^2 + (y1 - y2)^2)
            dist = math.sqrt((carx - wp_x)**2 + (cary - wp_y)**2)
            
            # This isn't entirely right.. need to make sure the waypoint is in front of the 
            # car as well as being the nearest
            if dist < min_dist:
                min_dist = dist
                min_loc = i

        closest_wp_pos = self.waypoints[min_loc].pose.pose.position     
        
        rospy.loginfo("Closeset waypoint- idx:%d x:%f y:%f", min_loc, closest_wp_pos.x, closest_wp_pos.y);
        
        # Now that we have the shortest distance, get the next LOOKAHEAD_WPS waypoints.
        # This next line ensures that we loop around to the start of the list if we've hit the end.
        # Not sure this is 100% correct... there's a pretty large delta between the positions 
        # at the end and beginning of the list 
        next_wps = list(islice(cycle(self.waypoints), min_loc, min_loc + LOOKAHEAD_WPS))

        rospy.loginfo("Publishing next waypoints to final_waypoints")
        lane = Lane()
        lane.waypoints = next_wps
        lane.header.stamp = rospy.Time(0)
        self.final_waypoints_pub.publish(lane)



if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
