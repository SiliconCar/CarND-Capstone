#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLight, TLStatus
from std_msgs.msg import Int32
from geometry_msgs.msg import TwistStamped

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
    2a. Set the speed based on if there's an upcoming red light
3. Publish this list to the final_waypoints publisher

'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_SPEED_METERS_PER_SEC = 10*0.447
SLOWDOWN_WPS = 50 # Number of waypoints before traffic light to start slowing down


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        """
        I set the queue_size to one for the /current_pose subscriber because I noticed
        that the pose messages recieved were lagging behind where the car was actually
        For example if the car crashed, it would take several seconds for the pose messages
        to catch up
        - Sean
        """
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1, buff_size=512*1024)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        #rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1, buff_size=512*1024)
        rospy.Subscriber('/all_traffic_waypoint',TLStatus,self.traffic_state_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=1)

        self.sim_testing = bool(rospy.get_param("~sim_testing", True))

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.waypoints = None
        self.current_pose = None
        self.current_velocity = None
        self.red_light_wp = -1
        self.last_wp_id = None
        self.next_light_state = None
        self.next_light_wp = None
        rospy.spin()

    def current_velocity_cb(self, msg):
        self.current_velocity = msg

    # Callback for the position updater topic
    def pose_cb(self, msg):
        self.current_pose = msg.pose
        # rospy.loginfo("WPUpdater: Car position updated to %s", self.current_pose)
        self.send_next_waypoints()

    # Callback for the waypoints updater topic
    def waypoints_cb(self, lane):
        # Looks like the waypoints won't change after they're initially sent to us. So once we get the data, we can ignore it later on
        # https://carnd.slack.com/archives/C6NVDVAQ3/p1504803165000409?thread_ts=1504718014.000557&cid=C6NVDVAQ3
        rospy.loginfo('WPUpdater: Got initial waypoints')
        if self.waypoints is None:
            self.waypoints = lane.waypoints
            self.send_next_waypoints()

    '''
    Callback for the traffic light topic. This message contains a single integer
    that represents the position of the closest waypoint to the traffic light
    '''
    def traffic_cb(self, light_idx):
        #rospy.loginfo("WPUpdater: Closest red traffic light at idx: %d", light_idx.data)
        self.red_light_wp = light_idx.data
        self.send_next_waypoints()

    def traffic_state_cb(self, tl_status):
        # Don't start off with a bad value
        if self.next_light_wp is None and tl_status.waypoint == -1:
            return
        self.next_light_wp = tl_status.waypoint
        self.next_light_state = tl_status.state
        if tl_status.state == TrafficLight.RED or tl_status.state == TrafficLight.YELLOW:
            self.red_light_wp = tl_status.waypoint
        else:
            self.red_light_wp = -1
        self.send_next_waypoints()

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
        if self.waypoints is None or self.current_velocity is None \
            or self.current_pose is None or self.next_light_wp is None or self.red_light_wp is None:
            return
        speed = self.current_velocity.twist.linear.x
        carx = self.current_pose.position.x
        cary = self.current_pose.position.y

        # rospy.loginfo("WPUpdater: Finding closest waypoint to car at position %f, %f", carx, cary)
        # find the closest waypoint to the car
        min_dist = sys.maxsize
        min_loc = None

        start_index = 0
        end_index = len(self.waypoints)

        
        if self.sim_testing and self.last_wp_id is not None:
            start_index = self.last_wp_id - 30
            end_index = min(end_index,self.last_wp_id + 30)

        #for i, waypoint in enumerate(self.waypoints):
        for i in range(start_index, end_index):
            waypoint = self.waypoints[i]
            wp_x = waypoint.pose.pose.position.x
            wp_y = waypoint.pose.pose.position.y
            # distance calulated with distance formula... sqrt((x1 - x2)^2 + (y1 - y2)^2)
            dist = math.sqrt((carx - wp_x)**2 + (cary - wp_y)**2)

            # This isn't entirely right.. need to make sure the waypoint is in front of the
            # car as well as being the nearest
            if dist < min_dist:
                min_dist = dist
                min_loc = i

        closest_wp = self.waypoints[min_loc]
        closest_wp_pos = closest_wp.pose.pose.position
        self.last_wp_id = min_loc
        # rospy.loginfo("WPUpdater: Closest waypoint- idx:%d x:%f y:%f", min_loc, closest_wp_pos.x, closest_wp_pos.y);
        # Now that we have the shortest distance, get the next LOOKAHEAD_WPS waypoints.
        # This next line ensures that we loop around to the start of the list if we've hit the end.
        next_wps = list(islice(cycle(self.waypoints), min_loc, min_loc + LOOKAHEAD_WPS - 1))
        '''
        Set the target speed of the vehicle based on traffic light locations.

        When there's no red light, all waypoints are set to the max speed of 4.47 m/s (10 mph)

        When there's a red light the waypoints speed are set as follows.
            i <= red light idx: linearly decrease speed from the vehicle's current speed to zero
            red light idx < i: MAX_SPEED

        -- Example --
        Assume the closest waypont is 25 and there's a red light at waypoint 29.
        The vehicle's speed is 10 m/s (MAX_SPEED = 10 m/s)

        The waypoints velocities will be as follows
        Waypoint: 25   26   27   28   29   30   31 ...
        Speed:    10   7.5  5    2.5  0    10   10

        '''
        current_velocity = self.get_waypoint_velocity(closest_wp)
        wp_delta = self.next_light_wp - min_loc

        is_light_ahead = wp_delta < SLOWDOWN_WPS
        is_red_light_ahead = (self.red_light_wp != -1
                              and is_light_ahead)
                              #and self.upcoming_light_state == TrafficLight.RED)
        # If this error is thrown, need to rework solution. This means that the traffic light waypoint
        # is behind the car. Hopefully this doesn't happen
        if is_red_light_ahead and self.red_light_wp <= min_loc:
            #rospy.loginfo("WPUpdater: Red light idx is behind closest waypoint idx. Need to rework our solution")
            return

        # slowdown_rate = (current_velocity/max(1, wp_delta))*0.9
        # rospy.loginfo('slowdown rate %f',slowdown_rate)
        slope = (MAX_SPEED_METERS_PER_SEC/SLOWDOWN_WPS)

        # Iterate through all the next waypoints and adjust their speed
        for i in range(len(next_wps) - 1):
            '''
            Covers 2 cases:
                - There's no red lights detected so all will be set to the max speed
                - There's a red light and we've already iterated through the points
                  before the traffic light. The remaining points (after the light)
                  should be set to max speed
            '''
            wp_to_go = self.next_light_wp - min_loc - i - 10 # Add buffer
            #out_vel = []
            if not is_red_light_ahead:
                if speed > 3*0.447 and wp_to_go < 30 and wp_to_go > 0:
                    self.set_waypoint_velocity(next_wps, i, 5*0.447)
                else:
                    self.set_waypoint_velocity(next_wps, i, MAX_SPEED_METERS_PER_SEC)
            # There's a red light and we're at a waypoint before the red light waypoint
            else: # Within 100 waypoints -> slow down:
                # Determine the velocity for this waypoint and set it
                if wp_to_go < 4:
                    target_vel = 0.0
                elif wp_to_go < 25:
                    target_vel = 5*0.447
                else:
                    target_vel = MAX_SPEED_METERS_PER_SEC - (SLOWDOWN_WPS - wp_to_go)*slope
                if target_vel < 0.1:
                    target_vel = 0
                # new_velocity = max(0, target_vel)
                #out_vel.append(target_vel)
                self.set_waypoint_velocity(next_wps, i, target_vel)

        #if out_vel:
        #    rospy.loginfo(','.join(str(_) for _ in out_vel))
        #self.red_light_wp = -1
        # rospy.loginfo("WPUpdater: Publishing next waypoints to final_waypoints")
        lane = Lane()
        lane.waypoints = next_wps
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        self.final_waypoints_pub.publish(lane)



if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
