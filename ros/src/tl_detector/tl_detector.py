#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from styx_msgs.msg import TLStatus
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math


STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb) # _extended

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        self.upcoming_traffic_light_pub = rospy.Publisher('/all_traffic_waypoint', TLStatus, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()
        # rospy.loginfo("The next traffic light state is %s and located at wp: %s", state, light_wp)

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if (state == TrafficLight.RED or state == TrafficLight.YELLOW) else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1


    def image_cb_extended(self, msg):
        """Identifies all traffic lights in the incoming camera image and publishes the index
            of the waypoint closest to the traffic light to /all_tl_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()
        # rospy.loginfo("The next traffic light state is %s with stop line at wp: %s", state, light_wp)

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            #light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            tl_status_msg = TLStatus()
            tl_status_msg.header.frame_id = '/world'
            tl_status_msg.header.stamp = rospy.Time(0)
            tl_status_msg.waypoint = light_wp
            tl_status_msg.state = self.state
            self.upcoming_traffic_light_pub.publish(tl_status_msg)
        else:
            # we keep publishing the last state and wp until it gets confirmed.
            tl_status_msg = TLStatus()
            tl_status_msg.header.frame_id = '/world'
            tl_status_msg.header.stamp = rospy.Time(0)
            tl_status_msg.waypoint = self.last_wp
            tl_status_msg.state = self.last_state
            self.upcoming_traffic_light_pub.publish(tl_status_msg)
        self.state_count += 1


    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        if self.waypoints is None:
            return
        #TODO implement
        min_dist = 10000
        min_loc = None

        pos_x = pose.position.x
        pos_y = pose.position.y
        # check all the waypoints to see which one is the closest to our current position
        for i, waypoint in enumerate(self.waypoints):
            wp_x = waypoint.pose.pose.position.x
            wp_y = waypoint.pose.pose.position.y
            dist = math.sqrt((pos_x - wp_x)**2 + (pos_y - wp_y)**2)
            if (dist < min_dist): #we found a closer wp
                min_loc = i     # we store the index of the closest waypoint
                min_dist = dist     # we save the distance of the closest waypoint

        # returns the index of the closest waypoint
        return min_loc

    def project_to_image_plane(self, point_in_world):
		"""Project point from 3D world coordinates to 2D camera image location
		Args:
		point_in_world (Point): 3D location of a point in the world
		Returns:
		x (int): x coordinate of target point in image
		y (int): y coordinate of target point in image
		"""

		fx = self.config['camera_info']['focal_length_x']
		fy = self.config['camera_info']['focal_length_y']
		image_width = self.config['camera_info']['image_width']
		image_height = self.config['camera_info']['image_height']
		cx = image_width/2
		cy = image_height/2

		# get transform between pose of camera and world frame
		trans = None
		rot = None
		x = 0
		y = 0
		try:
			now = rospy.Time.now()
			self.listener.waitForTransform("/base_link",
				  "/world", now, rospy.Duration(1.0))
			(trans, rot) = self.listener.lookupTransform("/base_link",
				  "/world", now)

		except (tf.Exception, tf.LookupException, tf.ConnectivityException):
			rospy.logerr("Failed to find camera to map transform")
			return None, None

		rpy = tf.transformations.euler_from_quaternion(rot)
		yaw = rpy[2]

		(ptx, pty, ptz) = (point_in_world.pose.pose.position.x, point_in_world.pose.pose.position.y, point_in_world.pose.pose.position.z)

		point_to_cam = (ptx * math.cos(yaw) - pty * math.sin(yaw),
				ptx * math.sin(yaw) + pty * math.cos(yaw), 
				ptz)
		point_to_cam = [sum(x) for x in zip(point_to_cam, trans)]

		##########################################################################################
		# DELETE THIS MAYBE - MANUAL TWEAKS TO GET THE PROJECTION TO COME OUT CORRECTLY IN SIMULATOR
		# just override the simulator parameters. probably need a more reliable way to determine if 
		# using simulator and not real car
		if fx < 10:
			fx = 2574
			fy = 2744
			point_to_cam[2] -= 1.0
			cx = image_height/2 + 70
			cy = image_height + 50
	  ##########################################################################################

		#rospy.loginfo_throttle(3, "camera to traffic light: " + str(point_to_cam))

		x = -point_to_cam[1] * fx / point_to_cam[0]; 
		y = -point_to_cam[2] * fy / point_to_cam[0]; 

		x = int(x + cx)
		y = int(y + cy) 

		#rospy.loginfo_throttle(3, "traffic light pixel (x,y): " + str(x) + "," + str(y))

		return (x, y)

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        x, y = self.project_to_image_plane(light)
	
	#only keep this code for debugging purpose
	#draw a small circle on the image, save the image and the point coordinates
	#cv2.circle(cv_image, (x,y), 25, thickness = 2)
	#filename = "/home/student/CarND-Captsone/imgs/trafficLights/TL-Img-X" + x + "_Y" + y + ".png"
	#cv2.imsave(filename, cv_image)

        #TODO use light location to zoom in on traffic light in image

        #Get classification

        #Until we develop the classifier, let's search light in self.lights (fed by sub3) and return light state
        light_state = None

    	for tl in self.lights:
    	    dist = math.sqrt((tl.pose.pose.position.x - light.position.x)**2 + (tl.pose.pose.position.y - light.position.y)**2)
            if (dist < 50): #means we found the light close to the stop line
                light_state = tl.state
	        break #no need to parse other lights once light was found

        return light_state
        #return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        closest_light_stop_wp = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
            # rospy.loginfo("Car position (at Wp index): %s", car_position)
        else:
            return -1, TrafficLight.UNKNOWN

        #TODO find the closest visible traffic light (if one exists)
        # we also want to make sure the traffic light is ahead of us (not behind)
        # we may need to add a threshold to ensure that we're not too far away in terms of waypoints
        # we use closest_waypoint for both car position and light positions

        #car position is the indice of the waypoint closest to the car.
        #light_positions include all the traffic light positions. We parse them one at a time.
        for light_stop_position in stop_line_positions:
    	    light_stop_pose = Pose()
    	    light_stop_pose.position.x = light_stop_position[0]
    	    light_stop_pose.position.y = light_stop_position[1]
    	    light_stop_wp = self.get_closest_waypoint(light_stop_pose)     #get the wp closest to each light_position
            if light_stop_wp >= car_position :    #it found a waypoint close to the traffic light and ahead of the car
                if closest_light_stop_wp is None:    #check if this is the first light we process
                    closest_light_stop_wp = light_stop_wp
                    light = light_stop_pose
                elif light_stop_wp < closest_light_stop_wp:
                    closest_light_stop_wp = light_stop_wp    #if we have a closer light_wp ahead of the car position, we allocate closer value
                    light = light_stop_pose

        if light:
            state = self.get_light_state(light)
            return closest_light_stop_wp, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
