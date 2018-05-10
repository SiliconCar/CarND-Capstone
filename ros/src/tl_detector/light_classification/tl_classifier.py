#uncomment the following in real test
from styx_msgs.msg import TrafficLight

import numpy as np
import cv2
import tensorflow as tf
import keras
from keras.models import Sequential
from keras.layers import Conv2D, MaxPooling2D
from keras.models import load_model
from PIL import Image
import os
import six.moves.urllib as urllib
from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt
import time
from glob import glob

cwd = os.path.dirname(os.path.realpath(__file__))

# Uncomment the following code if need to visualize the detection output
#os.chdir(cwd+'/models')
#from object_detection.utils import visualization_utils as vis_util


class TLClassifier(object):
    def __init__(self, threshold, hw_ratio, sim_testing):
        self.threshold = threshold
        self.hw_ratio = hw_ratio    #height_width ratio
        print('Initializing classifier with threshold =', self.threshold)
        self.signal_classes = ['Red', 'Green', 'Yellow']
       # self.signal_status = TrafficLight.UNKNOWN
        self.signal_status = None

        self.tl_box = None

        self.num_pixels = 25
        # Define red pixels in hsv color space
        self.lower_red_1 = np.array([0,  70, 50],   dtype = "uint8")
        self.upper_red_1 = np.array([10, 255, 255], dtype = "uint8")

        self.lower_red_2 = np.array([170,  70,  50], dtype = "uint8")
        self.upper_red_2 = np.array([180, 255, 255], dtype = "uint8")

        os.chdir(cwd)
        
        #if sim_testing, we use a detection and classification models
        #if site_testing, we use a single model which does both detection and classification
        
        if sim_testing: #we use different models for classification
            #keras classification model
            self.cls_model = load_model('tl_model_5.h5') #switched to model 5 for harsh light
            self.graph = tf.get_default_graph()

            #tensorflow localization/detection model
            detect_model_name = 'ssd_mobilenet_v1_coco_11_06_2017' #was 'ssd_inception_v2_coco_11_06_2017'
            PATH_TO_CKPT = detect_model_name + '/frozen_inference_graph.pb'
            # setup tensorflow graph
            self.detection_graph = tf.Graph()
            # configuration for possible GPU
            config = tf.ConfigProto()
            config.gpu_options.allow_growth = True
            # load frozen tensorflow detection model and initialize
            # the tensorflow graph
        
        else: # site testing where we load the localization+classification model        
            detect_model_name = 'ssd_kyle_v2'
            PATH_TO_CKPT = detect_model_name + '/frozen_inference_graph.pb'
            # setup tensorflow graph
            self.detection_graph = tf.Graph()
        
            # configuration for possible GPU use
            config = tf.ConfigProto()
            config.gpu_options.allow_growth = True
            # load frozen tensorflow detection model and initialize 
            # the tensorflow graph        
        
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
               serialized_graph = fid.read()
               od_graph_def.ParseFromString(serialized_graph)
               tf.import_graph_def(od_graph_def, name='')

            self.sess = tf.Session(graph=self.detection_graph, config=config)
            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
              # Each box represents a part of the image where a particular object was detected.
            self.boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
              # Each score represent how level of confidence for each of the objects.
              # Score is shown on the result image, together with the class label.
            self.scores =self.detection_graph.get_tensor_by_name('detection_scores:0')
            self.classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            self.num_detections =self.detection_graph.get_tensor_by_name('num_detections:0')

    # Helper function to convert image into numpy array
    def load_image_into_numpy_array(self, image):
         #(im_width, im_height, im_depth)= image.shape
         #print(image.shape)
         return np.asarray(image, dtype="uint8" )
         #np.array(image.getdata()).reshape(
         #   (im_height, im_width, 3)).astype(np.uint8)

    # Helper function to convert normalized box coordinates to pixels
    def box_normal_to_pixel(self, box, dim):

        height, width = dim[0], dim[1]
        box_pixel = [int(box[0]*height), int(box[1]*width), int(box[2]*height), int(box[3]*width)]
        return np.array(box_pixel)

    def get_localization(self, image, visual=False):
        """Determines the locations of the traffic light in the image

        Args:
            image: camera image

        Returns:
            list of integers: coordinates [x_left, y_up, x_right, y_down]

        """

        with self.detection_graph.as_default():
              image_expanded = np.expand_dims(image, axis=0)
              (boxes, scores, classes, num_detections) = self.sess.run(
                  [self.boxes, self.scores, self.classes, self.num_detections],
                  feed_dict={self.image_tensor: image_expanded})
# Uncomment the following code for visualizing detection output
#              if visual == True:
#                  vis_util.visualize_boxes_and_labels_on_image_array(
#                      image_np,
#                      np.squeeze(boxes),
#                      np.squeeze(classes).astype(np.int32),
#                      np.squeeze(scores),
#                      category_index,
#                      use_normalized_coordinates=True,min_score_thresh=.2,
#                      line_thickness=3)
#
#                  plt.figure(figsize=(9,6))
#                  plt.imshow(image_np)
#                  plt.show()

              boxes=np.squeeze(boxes)
              classes =np.squeeze(classes)
              scores = np.squeeze(scores)

              cls = classes.tolist()
              #print(cls)
              # Find the first occurence of traffic light detection id=10
              idx = next((i for i, v in enumerate(cls) if v == 10.), None)
              # If there is no detection
              if idx == None:
                  box=[0, 0, 0, 0]
                  print('no detection!')
              # If the confidence of detection is too slow, 0.3 for simulator
              elif scores[idx]<=self.threshold: #updated site treshold to 0.01 for harsh light
                  box=[0, 0, 0, 0]
                  print('low confidence:', scores[idx])
              #If there is a detection and its confidence is high enough
              else:
                  #*************corner cases***********************************
                  dim = image.shape[0:2]
                  box = self.box_normal_to_pixel(boxes[idx], dim)
                  box_h = box[2] - box[0]
                  box_w = box[3] - box[1]
                  ratio = box_h/(box_w + 0.01)
                  # if the box is too small, 20 pixels for simulator
                  if (box_h <20) or (box_w<20):
                      box =[0, 0, 0, 0]
                      print('box too small!', box_h, box_w)
                  # if the h-w ratio is not right, 1.5 for simulator, 0.5 for site
                  elif (ratio < self.hw_ratio):
                      box =[0, 0, 0, 0]
                      print('wrong h-w ratio', ratio)
                  else:
                       print(box)
                       print('localization confidence: ', scores[idx])
                 #****************end of corner cases***********************
              self.tl_box = box

        return box

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): cropped image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # Resize cropped

        #img_resize = cv2.resize(image, (32, 32))
        # Color map conversion
        #cv2.imwrite('JFKyleTest.jpg', image)
        img_resize=cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # Convert to four-dimension input as required by Keras
        img_resize = np.expand_dims(img_resize, axis=0).astype('float32')
        # Normalization
        img_resize/=255.
        # Prediction
        with self.graph.as_default():
            predict = self.cls_model.predict(img_resize)
            # Get color classification
            tl_color = self.signal_classes[np.argmax(predict)]
            # TrafficLight message
            self.signal_status = tl_color
        # uncomment the following in real test
        if tl_color == 'Red':
            self.signal_status = TrafficLight.RED
        elif tl_color == 'Green':
            self.signal_status = TrafficLight.GREEN
        elif tl_color == 'Yellow':
            self.signal_status = TrafficLight.YELLOW

        return self.signal_status

    # Main function for end-to-end bounding box localization and light color
    # classification
    def get_localization_classification(self, image, visual=False):  
        """Determines the locations of the traffic light in the image

        Args:
            image: camera image

        Returns:
            box: list of integer for coordinates [x_left, y_up, x_right, y_down]
            conf: confidence
            cls_idx: 1->Green, 2->Red, 3->Yellow

        """
        
        category_index={1: {'id': 1, 'name': 'Green'},
                        2: {'id': 2, 'name': 'Red'},
                        3: {'id': 3, 'name': 'Yellow'}}  
        
        with self.detection_graph.as_default():
              image_expanded = np.expand_dims(image, axis=0)
              (boxes, scores, classes, num_detections) = self.sess.run(
                  [self.boxes, self.scores, self.classes, self.num_detections],
                  feed_dict={self.image_tensor: image_expanded})
              '''
              if visual == True:
                  vis_util.visualize_boxes_and_labels_on_image_array(
                      image,
                      np.squeeze(boxes),
                      np.squeeze(classes).astype(np.int32),
                      np.squeeze(scores),
                      category_index,
                      use_normalized_coordinates=True,min_score_thresh=.2,
                      line_thickness=3)
    
                  plt.figure(figsize=(9,6))
                  plt.imshow(image)
                  plt.show()  
              '''
              
              boxes=np.squeeze(boxes)  # bounding boxes
              classes =np.squeeze(classes) # classes
              scores = np.squeeze(scores) # confidence
    
              cls = classes.tolist()
             
              # Find the most confident detection/classification
              idx = 0;
              conf = scores[idx]
              cls_idx = cls[idx]
              
              # If there is no detection
              if idx == None:
                  box=[0, 0, 0, 0]
                  print('no detection!')
                  cls_idx = 4.0
              # If the confidence of detection is too slow, 0.3 for simulator    
             
              #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
              elif scores[idx]<=0.15:
              #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!    
                  box=[0, 0, 0, 0]
                  print('low confidence:', scores[idx])
                  cls_idx = 4.0
              #If there is a detection and its confidence is high enough    
              else:
                  #*************corner cases***********************************
                  dim = image.shape[0:2]
                  box = self.box_normal_to_pixel(boxes[idx], dim)
                  box_h = box[2] - box[0]
                  box_w = box[3] - box[1]
                  ratio = box_h/(box_w + 0.01)
                  # if the box is too small, 20 pixels for simulator
                  if (box_h <10) or (box_w<10):
                      box =[0, 0, 0, 0]
                      cls_idx = 4.0
                      print('box too small!', box_h, box_w)
                  # if the h-w ratio is not right, 1.5 for simulator    
                  #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                  elif (ratio<1.0):
                  #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!    
                      box =[0, 0, 0, 0]
                      cls_idx = 4.0
                      print('wrong h-w ratio', ratio)
                  else:    
                       print(box)
                       print('localization confidence: ', scores[idx])
                 #****************end of corner cases***********************      
              self.tl_box = box
             
        return box, conf, cls_idx

if __name__ == '__main__':
        tl_cls =TLClassifier()
        os.chdir(cwd)
        TEST_IMAGE_PATHS= glob(os.path.join('traffic_light_images/', '*.jpg'))
        for image_path in TEST_IMAGE_PATHS:
            img_full = Image.open(image_path)
            img_full_np = tl_cls.load_image_into_numpy_array(img_full)
            img_full_np_copy = np.copy(img_full_np)
            print('Processing following file:',image_path)
            start = time.time()
            b = tl_cls.get_localization(img_full_np)
            end = time.time()
            print('Localization time: ', end-start)
            start = time.time()
            # If there is no detection or low-confidence detection
            if np.array_equal(b, np.zeros(4)):
               print ('unknown')
            else:
               img_np = cv2.resize(img_full_np_copy[b[0]:b[2], b[1]:b[3]], (32, 32))
               tl_cls.get_classification(img_np)
               print(tl_cls.signal_status)
            end = time.time()
            print('Classification time: ', end-start)
