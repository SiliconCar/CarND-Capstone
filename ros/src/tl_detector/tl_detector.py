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
import time
import numpy as np

STATE_COUNT_THRESHOLD = 3
SIM_TESTING = True


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
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1, buff_size=2*52428800) # extended

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        #we don't use this publisher anymore. We publish the status of all traffic lights. Not just the red ones.
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        self.upcoming_traffic_light_pub = rospy.Publisher('/all_traffic_waypoint', TLStatus, queue_size=1)
        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.count = 0  #processed images count
        self.total_classification = 0
        self.tp_classification = 0.0
        print("TL Detection...initiationalization complete")

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

        # rospy.loginfo('Image seq %s', msg.header.seq)
        img_time = msg.header.stamp
        age = rospy.Time.now() - img_time
        if age.secs > 0.2:
            rospy.loginfo('Image too old. Age %s', age.secs)
            light_wp, state = -1, TrafficLight.UNKNOWN
        else:
            self.camera_image = msg
            light_wp, state = self.process_traffic_lights()

        # rospy.loginfo("The next traffic light state is %s and located at wp: %s", state, light_wp)

        # '''
        # Publish upcoming red lights at camera frequency.
        # Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        # of times till we start using it. Otherwise the previous stable state is
        # used.
        # '''
        # if self.state != state:
        #     self.state_count = 0
        #     self.state = state
        # elif self.state_count >= STATE_COUNT_THRESHOLD:
        #     self.last_state = self.state
        #     light_wp = light_wp if (state == TrafficLight.RED or state == TrafficLight.YELLOW) else -1
        #     self.last_wp = light_wp
        #     self.upcoming_red_light_pub.publish(Int32(light_wp))
        # else:
        #     self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        # self.state_count += 1

        #self.camera_image = msg
        #light_wp, state = self.process_traffic_lights()
        #rospy.loginfo("TL_Detector: The next traffic light state is %s with stop line at wp: %s", state, light_wp)

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
            self.last_state = state
            #light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            tl_status_msg = TLStatus()
            tl_status_msg.header.frame_id = '/world'
            tl_status_msg.header.stamp = rospy.Time(0)
            tl_status_msg.waypoint = light_wp
            tl_status_msg.state = state
            # rospy.loginfo("Light Wp is %s and state is %s", light_wp, self.state)
            self.upcoming_traffic_light_pub.publish(tl_status_msg)
        else:
            # we keep publishing the last state and wp until it gets confirmed.
            tl_status_msg = TLStatus()
            tl_status_msg.header.frame_id = '/world'
            tl_status_msg.header.stamp = rospy.Time(0)
            tl_status_msg.waypoint = self.last_wp
            tl_status_msg.state = self.last_state
            #rospy.loginfo("Light Wp is %s and state is %s:", self.last_wp, self.last_state)
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

    def find_stop_line(self, closest_light_wp):
        """Find waypoint for stop line associated to traffic light
        Args:
            closest_light_wp: closest waypoint to the light
        Returns:
            closest_stop_line_wp: waypoint closest to the sop line before the light
        """
        stop_line_positions = self.config['stop_line_positions']
        closest_light_stop_wp = None

        #search  the stop line waypoint which is the closest to closest_light_wp
        min_dist = 10000
        for light_stop_position in stop_line_positions:
            #convert 2D position into Pose format to get closest waypoint
            light_stop_pose = Pose()
            light_stop_pose.position.x = light_stop_position[0]
            light_stop_pose.position.y = light_stop_position[1]
            light_stop_wp = self.get_closest_waypoint(light_stop_pose)
            dist = abs(closest_light_wp - light_stop_wp)
            if dist < min_dist :
                min_dist = dist
                closest_light_stop_wp = light_stop_wp #note that it can be a bit ahead of the stop line

        return closest_light_stop_wp

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

        #print("Image Size:", image_width, image_height)
        cx = image_width / 2
        cy = image_height / 2

        # get transform between pose of camera and world frame
        trans = None
        rot = None
        x = 0
        y = 0
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link", "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",          "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")
            return None, None

        if (trans and rot):
            rpy = tf.transformations.euler_from_quaternion(rot)
            yaw = rpy[2]

            (ptx, pty, ptz) = (point_in_world.position.x, point_in_world.position.y, point_in_world.position.z)

            #rotation
            point_to_cam = (ptx * math.cos(yaw) - pty * math.sin(yaw),
                            ptx * math.sin(yaw) + pty * math.cos(yaw),
                            ptz)
            #translation
            point_to_cam = [sum(x) for x in zip(point_to_cam, trans)]

            #project to plan
            x = -fx * point_to_cam[1]/point_to_cam[0]
            y = -fy * point_to_cam[2]/point_to_cam[0]

            x = int(x + cx)
            y = int(y + cy)

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
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        #x, y = self.project_to_image_plane(light)
        #print ("Traffic Light @:", x,y)
        #only keep this code for debugging purpose
        #draw a small circle on the image, save the image and the point coordinates
        #cv2.circle(cv_image, (int(x),int(y)), 10, (255,0,0), thickness = -1)
        #filename = "/home/student/CarND-Capstone/imgs/trafficlights/TL-Img-X" + str(x) + "_Y" + str(y) + ".png"
        #cv2.imwrite("/home/student/CarND-Capstone/imgs/trafficlights/JFImage.png", cv_image)


        #TODO use light location to zoom in on traffic light in image
        #Prepare image for classification
        if SIM_TESTING: #we cut 50 pixels left and right of the image and the bottom 100 pixels
            width, height, _ = cv_image.shape
            x_start = int(width * 0.10)
            x_end = int(width * 0.90)
            y_start = 0
            y_end = int(width * 0.85) 
            processed_img = cv_image[x_start:x_end, y_start:y_end]
        else:
            x_projected, y_projected = self.project_to_image_plane(light)
            print("X, Y projected:", x_projected, y_projected)
            #we still need to zoom on the traffic light
            processed_img = cv_image.copy()

        #Convert image to RGB format
        processed_img = cv2.cvtColor(processed_img, cv2.COLOR_BGR2RGB)
        
        #Get classification

        #initialize light_state to unknown by default
        light_state = TrafficLight.UNKNOWN
        light_state_via_msg = None

        #get the ground truth traffic light states through the traffic light messages
        for tl in self.lights:
            dist = math.sqrt((tl.pose.pose.position.x - light.position.x)**2 + (tl.pose.pose.position.y - light.position.y)**2)
            if (dist < 50): #means we found the light close to the stop line
                light_state_via_msg = tl.state
                break #no need to parse other lights once light was found

        #detect traffic light position (box) in image
        #convert image to np array
        img_full_np = self.light_classifier.load_image_into_numpy_array(processed_img)
        b = self.light_classifier.get_localization(img_full_np)
        print(b)
        # If there is no detection or low-confidence detection
        unknown = False
        if np.array_equal(b, np.zeros(4)):
           print ('unknown')
           unknown = True
        else:    #we can use the classifier to classify the state of the traffic light
           img_np = cv2.resize(processed_img[b[0]:b[2], b[1]:b[3]], (32, 32))
           self.light_classifier.get_classification(img_np)
           light_state = self.light_classifier.signal_status

        rospy.loginfo("Upcoming light %s, True state: %s", light_state, light_state_via_msg)

        #compare detected state against ground truth
        if not unknown:
            self.count = self.count + 1
            filename = "sim_image_" + str(self.count)
            if (light_state == light_state_via_msg):
               self.tp_classification = self.tp_classification + 1
               #filename = filename + "_good_" + str(light_state) + ".jpg"
            else:
               filename = filename + "_bad_" + str(light_state) + ".jpg"
            #    cv2.imwrite(filename, cv_image)
            self.total_classification = self.total_classification + 1
            accuracy = (self.tp_classification / self.total_classification) * 100
            if self.count % 20 == 0:
                rospy.loginfo("Classification accuracy: %s", accuracy)
        #return light_state_via_msg
        return light_state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        closest_light_wp = None
        closest_light_stop_wp = None
        dist_to_light = 10000   #initialize to high value

        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
            #rospy.loginfo("Car position (at Wp Index): %s", car_position)
        else:
            return -1, TrafficLight.UNKNOWN

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

        if (car_position and closest_light_stop_wp):
            dist_to_light = abs(car_position - closest_light_stop_wp)
            #rospy.loginfo("Closest light position (in Wp index): %s", closest_light_stop_wp)

        if light and dist_to_light < 100:       #we check the status of the traffic light if it's within 200 waypoints distance
            state = self.get_light_state(light)
            #closest_stop_line_wp = self.find_stop_line(closest_light_wp)
            return closest_light_stop_wp, state
        #self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
