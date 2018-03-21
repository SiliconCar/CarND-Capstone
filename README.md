# Udacity Self-Driving Car Engineer Nanodegree

* * *


## Final Project - System Integration

## Team SiliconCar

![image alt text](imgs/readme_image_0.jpg)

The System Integration project is the final project of the Udacity Self-Driving Car Engineer Nanodegree. As a team, we built ROS nodes to implement core functionality of the autonomous vehicle system, including traffic light detection, control, and waypoint following. This software system will be deployed on Carla (Udacity’s Self Driving Lincoln MKZ) to autonomously drive it around a test track.

## The Team

### Jean-François Vanreusel

jfvanreu@gmail.com

Director of Globalization at Adobe in San Jose, CA.

Project Contributions: Team Lead, traffic light detector and classification controller.

### Doug Guastaferro

[dguastaferro@gmail.com](mailto:dguastaferro@gmail.com)

Software Engineering Manager based in San Francisco. Amateur mechanic & DIY enthusiast.

Project contributions: Waypoint publisher, node communication infrastructure, callback for traffic light publisher

### Thomas Antony

[tantony.purdue@gmail.com](mailto:tantony.purdue@gmail.com)

PhD Candidate at Purdue University

Project Contributions: 

Implemented and tuned PID Controller for throttle, proportional controller for brakes, path/velocity planning for stopping at traffic lights, testing traffic light detector on GPU, performance tuning of ROS communication system to reduce lag.

### Kyle Guan

Email: kcguan@gmail.com

Research Scientist at Nokia Bell Labs based in Holmdel, New Jersey

Project Contributions: Train and evaluate deep-learning models for traffic light detection and traffic-light color classification to achieve the best trade-off between accuracy and computation time.

### Sean Scheideman

searn@ualberta.ca

MSc Student at the University of Alberta

Project Contributions: Setup twist controller to follow waypoints, stopping based off of traffic light state, testing on real bag files.

## Software Architecture

*Note: For this project, the obstacle detection node is not implemented*![image alt text](imgs/readme_image_1.png)

## Perception Subsystem

This subsystem reads the world surrounding the vehicle and publishes relevant information to other subsystems. Specifically, this subsystem determines the state of upcoming traffic lights and publishes their status to other subsystems.

### Traffic Light Detection Node

A crucial part of the vehicle’s self-driving capabilities comes from the ability to detect and classify upcoming traffic lights. This node processes images provided by the vehicle’s onboard camera and publishes upcoming traffic light information to the /traffic_waypoint topic. The Waypoint Updater node uses this information to determine if/when the car should slow down to safely stop at upcoming red lights.  We initially took a two-stage deep learning based approach for both simulator and test site testing. That is, the traffic light detection module consists of two CNN based models: traffic light detection (localization) and (light) color classification. However, due to challenging light conditions (brightness, sun exposure), we decided to create a single shot localizer-classifier solution for the test site testing.

#### Traffic Light Detection (Simulator Only)

Traffic light detection takes a captured image as input and produces the bounding boxes as the output to be fed into the classification model. After many trial-and-errors, we decided to use [TensorFlow Object Detection API](https://github.com/tensorflow/models/tree/master/research/object_detection), which is an open source framework built on top of TensorFlow to construct, train and deploy object detection models. The Object Detection API also comes with a collection of detection models pre-trained on the [COCO dataset](http://mscoco.org/) that are well suited for fast prototyping. Specifically, we use a lightweight model: ssd_mobilenet_v1_coco that is based on Single Shot Multibox Detection (SSD) framework with minimal modification. The COCO dataset contains images of 90 classes ranging from vehicle to human. The index for traffic light is 10. Though this is a general-purpose detection model (not optimized specifically for traffic light detection), we find this model sufficiently met our needs, achieving the balance between good bounding box accuracy (as shown in the following figure) and fast running time.

![image alt text](imgs/readme_image_2.png)

The traffic-light detection is implemented in get_localization(self, image, visual=False) function in CarND-Capstone/tree/master/ros/src/tl_detector/light_classification/tl_classifier.py. The traffic-light detector drops any camera frames that are older than 0.2 seconds to overcome any lag in the system.

#### Traffic Light Classification (Simulator Only)

After locating the bounding box for the traffic light, we crop the image to only include the traffic light, resize it to 32x32, and pass this along to the object classification step, as shown in the following figure. 

![image alt text](imgs/readme_image_3.png)

We use a simple CNN for our classification. It consists of three convolutional layer with (3x3 kernel), followed by a max_pooling layer, a flatten layer, and two fully connected layers. The architectural details of the CNN are shown in the following:

![image alt text](imgs/readme_image_4.png)

For training images, we use a combination of web scraping, simulator image capturing, and ROSbag image capturing. We use Keras (with Tensorflow as backend) for training with a total of 2,000 images (10% of which as validation/test samples). Except for normalization, we do not use other image augmentation techniques. We trained for 25 epochs and can achieve >99% validation/test accuracy. We save the model into the .h5 file.  The traffic-light classification is implemented in get_classification(self, image) function in CarND-Capstone/tree/master/ros/src/tl_detector/light_classification/tl_classifier.py.

Udacity provided a rosbag with test site images so we could train our detection and classification algorithm. We extracted the images from the rosbag as follows:

 - *Step 1:* Open a terminal window:
      source devel/setup.sh
      roscore
 - *Step 2:* Open another terminal window:
      source devel/setup.sh
      rosbag play -l traffic_light_bag_files/<rosbag_name>.bag
 - *Step 3:* Open a third terminal window:
      source devel/setup.sh
      rostopic list //to identify the current topics
      rosrun image_view image_saver image:=/topic  //where topic is the topic we want to listen to.
      
Images are then extracted from the rosbag and saved in the current folder.

#### Performance Tuning

The traffic light detection system was initially found to be always lagging 2-3 seconds behind the simulator despite improvements in prediction time and setting the queue size to one. This was due to a bug in how ROS processes large messages. The camera image subscriber was modified to have a large buffer size (52 Mb) and a queue size of one. 

Another possible bottleneck was found to be in the very-first image processing step which was somehow very slow compared to the later ones. To overcome this, during the initialization of the tl_detector node, the traffic light localizer/classifier is called with an empty 800x600 image ensuring that all the TensorFlow systems are initialized. This ensured that subsequent evaluations were much faster.

#### Single-shot localization-classification solution (test site only)
In our previous submission, we took a two-stage deep learning based approach in traffic light classification. That is, with two separate modules in tandem, we first detect (localize) the traffic light(s) in an image (frame) (localization) and then input the cropped traffic light image to a CNN based classifier. For the detector part, we used TensorFlow Object Detection API, specifically ssd_mobilenet_v1_coco (developed with Single Shot Multibox Detection (SSD) framework and COCO image datasets). During our initial submission, we found this model sufficiently met our needs, achieving the balance between good box accuracy (with the lighting condition and camera configuration at during September and October 2017) and fast running time.

However, in the subsequent submissions, we found out the ssd_mobilenet_v1_coco as a general-purpose detection model (not optimized specifically for traffic light detection), does not generalize well when the lighting condition deviates from that of the training images. For example, the very saturated light condition (with glare and reflection, as shown in the following test site Images from late November 2017) could easily throw off the detector. This led to miss-detection, false-positive, or inaccurate bounding boxes, which in turn gave rise to erroneous classification results.

![image alt text](imgs/readme_image_6.png)

We tried some tricks to alleviate the hard lighting conditions, such as **Gamma Correction technique** and cropping. Though these approaches reduced the miss-detection and improve bounding box accuracy to a certain degree (as shown in the following figure), it also could lead to overfitting or an increase in false-positives, should the lighting condition at the test site change again.

![image alt text](imgs/readme_image_7.png)

To address the above mentioned issues, we decided to make two design changes in this iteration:
 1.	Instead of using a general purpose detector, we optimized the ssd_mobilenet_v1_coco specific for traffic light detection (in the test site).
 2.	Instead of a two-stage approach, we opt for an end-to-end one-shot approach that provide both bounding box and classification results as outputs.

To this end, we selected a total about 300 test site images with various lighting conditions and camera configurations. We manually label the bounding box and light colors using labelImg. That is, we optimized an SSD detector for localizing and classifying Red, Green, and Yellow traffic light. We follow steps listed in the blog post [Tracking Custom Objects Intro - Tensorflow Object Detection API Tutorial] (https://pythonprogramming.net/custom-objects-tracking-tensorflow-object-detection-api-tutorial/) and trained for >12,000 global steps (this on average took about 12 hours on a MacBook Pro). Our initial assessments show that the one-shot detector can perform very well even in some very challenging lighting conditions, as shown in the following.

![image alt text](imgs/readme_image_8.png)
![image alt text](imgs/readme_image_9.png)
![image alt text](imgs/readme_image_10.png)
![image alt text](imgs/readme_image_11.png)

## Planning Subsystem

The planning subsystem plans the vehicle’s path based on the vehicle’s current position and velocity along with the state of upcoming traffic lights. A list of waypoints to follow is passed on to the control subsystem.

### Waypoint Loader Node

This node was implemented by Udacity. It loads a CSV file that contains all the waypoints along the track and publishes them to the topic /base_waypoints. The CSV can easily be swapped out based on the test location (simulator vs real world).

### Waypoint Updater Node

The bulk of the path planning happens within this node. This node subscribes to three topics to get the entire list of waypoints, the vehicle’s current position, and the state of upcoming traffic lights. Once we receive the list of waypoints, we store this result and ignore any future messages as the list of waypoints won’t change. This node publishes a list of waypoints to follow - each waypoint contains a position on the map and a target velocity.

Every time the vehicle changes its position, we plan a new path. First, we find the closest waypoint to the vehicle’s current position and build a list containing the next 200 waypoints. Next, we look at the upcoming traffic lights. If there are any upcoming red lights, we adjust the speed of the waypoints immediately before the traffic light to slow the vehicle to a halt at the red light. The vehicle speed is controlled by the following rules:

1. If the car is within 30 waypoints of a green light, we slow to 5 mph. The reasoning behind this is that a green light lasts only 2-3 seconds in the simulator and we would not make it past the light on time if we kept going at full speed anyway.

2. If the car is within 50 waypoints of a red/yellow light, start slowing down with a linearly interpolated speed that would get it to a stop at the light.

3. If the car is within than 25 waypoints of a red/yellow light, we slow to 5 mph.

4. If the car is less than 4 waypoints from the stop-line for the light, command a full stop.

5. In all other cases, command the max-speed of 10 mph

The final list of waypoints is published on the /final_waypoints topic.

## Control Subsystem

This subsystem publishes control commands for the vehicle’s steering, throttle, and brakes based on a list of waypoints to follow. 

### Waypoint Follower Node

This node was given to us by Udacity. It parses the list of waypoints to follow and publishes proposed linear and angular velocities to the /twist_cmd topic

### Drive By Wire (DBW) Node

The DBW node is the final step in the self driving vehicle’s system. At this point we have a target linear and angular velocity and must adjust the vehicle’s controls accordingly. In this project we control 3 things: throttle, steering, brakes. As such, we have 3 distinct controllers to interface with the vehicle.

#### Throttle Controller

The throttle controller is a simple PID controller that compares the current velocity with the target velocity and adjusts the throttle accordingly. The throttle gains were tuned using trial and error for allowing reasonable acceleration without oscillation around the set-point.

#### Steering Controller

This controller translates the proposed linear and angular velocities into a steering angle based on the vehicle’s steering ratio and wheelbase length. To ensure our vehicle drives smoothly, we cap the maximum linear and angular acceleration rates. The steering angle computed by the controller is also passed through a low pass filter to reduce possible jitter from noise in velocity data.

#### Braking Controller

This is the simplest controller of the three - we simply proportionally brake based on the difference in the vehicle’s current velocity and the proposed velocity. This proportional gain was tuned using trial and error to ensure reasonable stopping distances while at the same time allowing low-speed driving. Despite the simplicity, we found that it works very well.

## Testing on Simulator

We can first validate our project in the simulator using the following commands:

git clone https://github.com/SiliconCar/CarND-Capstone.git

Then, install python dependencies

cd CarND-Capstone
pip install -r requirements.txt

We ran our simulator testing using Docker.
To launch the Docker environment, follow the instructions below:

cd CarND-Capstone
./run.sh

To build the environment in Docker, type in the following instruction (in the CarND-Captson directory):
./udacity_make

Finally, we can run our self-driving car code along with the simulator

cd ros

catkin_make (only necessary if we want to rebuild the code)

source devel/setup.sh

roslaunch launch/styx.launch

Run the simulator (which can be downloaded here: 
https://github.com/udacity/CarND-Capstone/releases/tag/v1.2

## Testing for Site

Most of the testing was done using the Udacity simulator. But we also tested our system on the site rosbag files, using the provided site.launch file. For example: 

#launch site launch file
roslaunch ros/launch/site.launch 
#play bag file 
rosbag play -l <bag file path> 

Also we tested our detection system by outputting the detected bounding boxes to a new topic /detections and viewing them in the rqt GUI. The code for publishing the bounding box images was not included in our submission to reduce latency but can be found on the branch sean-bag-testing in our github repository.

![image alt text](imgs/readme_image_5.png)

## Known Issues

1. Due to the 10mph speed restriction and the timing of traffic lights, the car may not successfully make it through the intersection before the light turns red again. This can be solved by adjusting the light timing or increasing the speed limit. 

