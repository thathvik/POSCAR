#!/usr/bin/env python
#
# Author:       Smrithi Reddy Thudi
# Date:         03-12-2020
# Description:  Continuously send images from gazebo to dope and save the dope files in a folder for looking at them later.
# Usage:        roslaunch triple_s_util video_to_dope

import rospy
import sys
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import yaml


class VideoToDope:
    def __init__(self):
        # Load the parameters from the parameter server
	self.path = rospy.get_param('~path') # to store camera config/yaml file
        self.camera_raw_topic = rospy.get_param('~camera_raw_topic')
        self.camera_info_topic = rospy.get_param('~camera_info_topic')
        

        self.bridge = CvBridge()
	# subscribe to Camera topics. 
        camera_raw = rospy.Subscriber(self.camera_raw_topic, Image, self.videoReceivedCallback)
        self.camera_info = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.cameraInfoCallback)
        
	#subcribe to dope topic 
        self.dope_subscriber = rospy.Subscriber(rospy.get_param('~dope_topic_subscribe'), Image, self.imageReceived)
	
	#publish to dope topic        
	self.dope_publisher = rospy.Publisher(rospy.get_param('~dope_topic_publish'), Image, queue_size=10, latch=True)
        self.dope_info_publisher = rospy.Publisher(rospy.get_param('~dope_topic_publish_info'), CameraInfo, queue_size=10, latch=True)

        # Load the config for the camera
        self.yaml_config = yaml.load(file(os.path.join(self.path_origin, rospy.get_param('~camera_info_filename'))).read())
      

	rospy.spin()

    def videoReceivedCallback(self, image):
        """ Convert an sensor_msgs/Image to an .jpeg file """
        self.currentImage = image;
	self.sendImage(image)


    def sendImage(self, image_name):
        """ Send an image to the publisher topic """
	#find yaml file!
        
        print 'Publishing image: ', image_name
        self.yaml_config.header = image_name.header
        self.dope_info_publisher.publish(self.yaml_config)
        self.dope_publisher.publish(image_name)


    def cameraInfoCallback(self, cameraInfo):
        """ Store the camera info to a file, this is only done once """
        file_to_save = file(os.path.join(self.path, rospy.get_param('~camera_info_output')), 'w')
        
        if yaml.dump(cameraInfo, file_to_save):
            self.camera_info.unregister()
            print 'Saved camera info'
        
        file_to_save.close()

    def imageReceived(self, image):
        """ Convert an sensor_msgs/Image to an .jpeg file """
        print 'Received an image'

        try:
            # Convert ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError, e:
            print(e)
        else:
            # Save OpenCV2 image as a jpeg
            image_path = os.path.join(self.path_save, self.current_image)

            if not cv2.imwrite(os.path.join(image_path), cv2_img):
                print 'Could not save image to ', image_path
            else:
                self.received_images = self.received_images + 1
                print 'Saved image as: ', self.current_image
                print 'Progress: %d/%d' % (self.received_images, self.total_images)
            













if __name__ == '__main__':
    rospy.init_node('video_to_dope', anonymous=True)

    VideoToDope()
