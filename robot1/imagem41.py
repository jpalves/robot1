#!/usr/bin/env python
#JPAlves 2023

import sys
import rclpy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, CompressedImage
import message_filters
from rclpy.qos import qos_profile_sensor_data, QoSProfile

class Rectifica:	
	def __init__(self,args=None): #modificar
		global node
		
		qos_profile = QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=5)
                                          
		rclpy.init(args=args)
		node = rclpy.create_node('retificador')
		camera_info_topic = "/color/camera_info"
		
		self.image_sub   =  message_filters.Subscriber(node,CompressedImage,'/color/image_raw/compressed', qos_profile=qos_profile)
		self.image_info  =  message_filters.Subscriber(node,CameraInfo,camera_info_topic, qos_profile=qos_profile)
		self.ts = message_filters.Cache(self.image_sub, 10)
		self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.image_info],100,0.001)
		self.ts.registerCallback(self.callback)
		
	def callback(self,rgb_msg,camera_info):
		rgb_image       = np.frombuffer(bytes(rgb_msg.data), np.uint8)
		rgb_image       = cv2.imdecode(rgb_image,cv2.IMREAD_COLOR)
		
		#print(camera_info)
		camera_info_K_l = np.array(camera_info.k).reshape([3, 3])
		camera_info_D   = np.array(camera_info.d)
		rgb_undist1     = cv2.undistort(rgb_image, camera_info_K_l, camera_info_D)
		cv2.imshow('Imagem',rgb_undist1)
		

		if cv2.waitKey(1) & 0xFF == ord(' '):
      			print('asneira')
    
if __name__ == '__main__':
   global node
   
   ic = Rectifica()
   try:
   	while rclpy.ok():
   		rclpy.spin_once(node)
   	
   	g_node.destroy_node()
   	rclpy.shutdown()
   except KeyboardInterrupt:
        print("Shutting down")

