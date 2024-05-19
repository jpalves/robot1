#!/usr/bin/env python3
#JPAlves 2024

import sys
import rclpy
import cv2
import math
import numpy as np
import statistics as s

import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, CompressedImage
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R
from ultralytics import YOLO
import torch

kpt_color_map = {
    5:{'name':'Right Shoulder', 'color':[193, 182, 255], 'radius':10},  
    6:{'name':'Left Shoulder', 'color':[193, 182, 255], 'radius':10},   
    7:{'name':'Right Elbow', 'color':[16, 144, 247], 'radius':10},      
    8:{'name':'Left Elbow', 'color':[16, 144, 247], 'radius':10},       
    9:{'name':'Right Wrist', 'color':[1, 240, 255], 'radius':10},       
    10:{'name':'Left Wrist', 'color':[1, 240, 255], 'radius':10},       
    11:{'name':'Right Hip', 'color':[140, 47, 240], 'radius':10},       
    12:{'name':'Left Hip', 'color':[140, 47, 240], 'radius':10},        
    13:{'name':'Right Knee', 'color':[223, 155, 60], 'radius':10},      
    14:{'name':'Left Knee', 'color':[223, 155, 60], 'radius':10},       
    15:{'name':'Right Ankle', 'color':[139, 0, 0], 'radius':10},        
    16:{'name':'Left Ankle', 'color':[139, 0, 0], 'radius':10},         
}

kpt_labelstr = {
    'font_size':4,             
    'font_thickness':1,       
    'offset_x':0,             
    'offset_y':150,            
}


skeleton_map = [
    {'srt_kpt_id':15, 'dst_kpt_id':13, 'color':[0, 100, 255], 'thickness':5},       
    {'srt_kpt_id':13, 'dst_kpt_id':11, 'color':[0, 255, 0], 'thickness':5},      
    {'srt_kpt_id':16, 'dst_kpt_id':14, 'color':[255, 0, 0], 'thickness':5},         
    {'srt_kpt_id':14, 'dst_kpt_id':12, 'color':[0, 0, 255], 'thickness':5},         
    {'srt_kpt_id':11, 'dst_kpt_id':12, 'color':[122, 160, 255], 'thickness':5},     
    {'srt_kpt_id':5, 'dst_kpt_id':11, 'color':[139, 0, 139], 'thickness':5},        
    {'srt_kpt_id':6, 'dst_kpt_id':12, 'color':[237, 149, 100], 'thickness':5},      
    {'srt_kpt_id':5, 'dst_kpt_id':6, 'color':[152, 251, 152], 'thickness':5},       
    {'srt_kpt_id':5, 'dst_kpt_id':7, 'color':[148, 0, 69], 'thickness':5},          
    {'srt_kpt_id':6, 'dst_kpt_id':8, 'color':[0, 75, 255], 'thickness':5},          
    {'srt_kpt_id':7, 'dst_kpt_id':9, 'color':[56, 230, 25], 'thickness':5},         
    {'srt_kpt_id':8, 'dst_kpt_id':10, 'color':[0,240, 240], 'thickness':5},         
]

class Quaternion:
    w: float
    x: float
    y: float
    z: float

    def quaternion_from_euler(self,roll, pitch, yaw):
     """
     Converts euler roll, pitch, yaw to quaternion
     """
     cy = math.cos(yaw * 0.5)
     sy = math.sin(yaw * 0.5)
     cp = math.cos(pitch * 0.5)
     sp = math.sin(pitch * 0.5)
     cr = math.cos(roll * 0.5)
     sr = math.sin(roll * 0.5)

     q = Quaternion()
     q.w = cy * cp * cr + sy * sp * sr
     q.x = cy * cp * sr - sy * sp * cr
     q.y = sy * cp * sr + cy * sp * cr
     q.z = sy * cp * cr - cy * sp * sr
     return q
      
    def yaw_from_quaternion(self, quaternion):
     x = quaternion.x
     y = quaternion.y
     z = quaternion.z
     w = quaternion.w

     siny_cosp = 2 * (w * z + x * y)
     cosy_cosp = 1 - 2 * (y * y + z * z)
     yaw = np.arctan2(siny_cosp, cosy_cosp)

     return yaw
MIN_MATCH_COUNT = 10

#os.environ["CUDA_VISIBLE_DEVICES"]="1"
VFOV = 57
HFOV = 86

class YoloROS:	
	def __init__(self,args=None):
		global node
		
		qos_profile = QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=5)
                                          
		rclpy.init(args=args)
		node = rclpy.create_node('YoloV8')
		camera_info_topic = "/color/camera_info"
		depth_info_topic  = "/aligned_depth_to_color/camera_info"
		self.model = YOLO('yolov8s-pose.pt')
		self.classes = self.model.names
		self.device  = 'cuda' if torch.cuda.is_available() else 'cpu'
		self.image_sub  = message_filters.Subscriber(node,CompressedImage,'/color/image_raw/compressed', qos_profile=qos_profile)
		self.image_info = message_filters.Subscriber(node,CameraInfo,camera_info_topic, qos_profile=qos_profile)
		self.depth_sub   =  message_filters.Subscriber(node,CompressedImage,'/aligned_depth_to_color/image_raw/compressedDepth', qos_profile=qos_profile)
		self.depth_info  =  message_filters.Subscriber(node,CameraInfo,depth_info_topic, qos_profile=qos_profile)
		self.ts = message_filters.Cache(self.image_sub, 10)
		self.tf_broadcaster = TransformBroadcaster(node)
		self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.image_info, self.depth_sub,self.depth_info],100,0.1)
		
		self.ts.registerCallback(self.callback)
		
	def score_frame(self, frame):
		"""
		Takes a single frame as input, and scores the frame using yolo model.
		:param frame: input frame in numpy/list/tuple format.
		:return: Labels and Coordinates of objects detected by model in the frame.
		"""
		self.model.to(self.device)
		frame = [frame]
		
		results = self.model(frame)
		labels, cord = results[0].boxes.cls.cpu().numpy(), results[0].boxes.xyxyn.cpu().numpy()
		
		return labels, cord, results[0].keypoints, len(results[0].boxes.cls)
	
	def plot_boxes(self, results, frame, rgb_msg, camera_info,depth, depth_info):
		"""
		Takes a frame and its results as input, and plots the bounding boxes and label on to the frame.
		:param results: contains labels and coordinates predicted by model on the given frame.
		:param frame: Frame which has been scored.
		:return: Frame with bounding boxes and labels ploted on it.
		"""
		
		labels, cord, keypoints, num_bbox = results
		bboxes_keypoints = keypoints.data.cpu().numpy().astype('uint32')
		
		
		x_shape, y_shape = frame.shape[1], frame.shape[0]
		for idx in range(num_bbox):
			bbox_keypoints = bboxes_keypoints[idx]
			for skeleton in skeleton_map:
				srt_kpt_id = skeleton['srt_kpt_id']
				srt_kpt_x = bbox_keypoints[srt_kpt_id][0]
				srt_kpt_y = bbox_keypoints[srt_kpt_id][1]
				
				dst_kpt_id = skeleton['dst_kpt_id']
				dst_kpt_x = bbox_keypoints[dst_kpt_id][0]
				dst_kpt_y = bbox_keypoints[dst_kpt_id][1]
				
				#skeleton_color = skeleton['color']
				#skeleton_thickness = skeleton['thickness']
				
				row = cord[idx]
				if labels[idx] == 0.0:
					if dst_kpt_x == 0 or dst_kpt_y == 0 or srt_kpt_x == 0 or srt_kpt_y == 0:
						continue
				
					keypoints_2d = keypoints.xy.cpu().numpy().astype('int32')[idx]
					
					u = np.array(keypoints_2d[:, 1]).clip(0, depth_info.height - 1)
					v = np.array(keypoints_2d[:, 0]).clip(0, depth_info.width  - 1)
					
					z = depth[u, v]
					k = depth_info.k
					px, py, fx, fy = k[2], k[5], k[0], k[4]
					x = z * (v - px) / fx
					y = z * (u - py) / fy
					points_3d = np.dstack([x, y, z]).reshape(-1, 3)*0.0010000000474974513
					
					points = points_3d[points_3d[:,0] != 0,:]
					if len(points) < 4 or keypoints_2d[12, 0] - keypoints_2d[11, 0] == 0:
						continue
					
					#W = camera_info.k[2]
					#H = camera_info.k[5]

					if (points_3d[12, 0] - points_3d[11, 0]):
						m = (points_3d[12, 2] - points_3d[11, 2])/(points_3d[12, 0] - points_3d[11, 0])
					else:
						m = 1.5708

					xxx,yyy = float(keypoints_2d[12, 0] + keypoints_2d[11, 0])/2 , float(keypoints_2d[12, 1] + keypoints_2d[11, 1])/2			
					angulo = math.pi - m
				
					self.draw_axis(frame, angulo, 0, 0,xxx,yyy)
					
					xx = s.median(points[:, 0])
					yy = s.median(points[:, 1])
					zz = s.median(points[:, 2])
					quat      = Quaternion()
					q = quat.quaternion_from_euler(0, angulo, 0)
					tf_ = TransformStamped()
					
					tf_.header = rgb_msg.header
					tf_.transform.translation.x =  xx
					tf_.transform.translation.y =  yy 
					tf_.transform.translation.z =  zz 
					
					tf_.transform.rotation.x = q.x
					tf_.transform.rotation.y = q.y
					tf_.transform.rotation.z = q.z
					tf_.transform.rotation.w = q.w
					tf_.child_frame_id = 'pessoa_'+ str(idx)
					
					self.tf_broadcaster.sendTransform(tf_)
					x1, y1, x2, y2 = int(row[0]*x_shape), int(row[1]*y_shape), int(row[2]*x_shape), int(row[3]*y_shape)
					#frame = cv2.line(frame, (srt_kpt_x, srt_kpt_y),(dst_kpt_x, dst_kpt_y),color=skeleton_color,thickness=skeleton_thickness)
					bgr = (0, 255, 0)
					cv2.rectangle(frame, (x1, y1), (x2, y2), bgr, 2)
					#cv2.putText(frame,"Pessoa", (x1+20, y1+20), cv2.FONT_HERSHEY_SIMPLEX, 0.9, bgr, 2)
					cv2.putText(frame, "Angulo: "+str(angulo), (x1+20, y1+40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, bgr, 2)
					
		return frame
	
	
	def draw_axis(self, img, yaw, pitch, roll, tdx=None, tdy=None, size = 50):
		if tdx != None and tdy != None:
			tdx = tdx
			tdy = tdy
		else:
			height, width = img.shape[:2]
			tdx = width / 2
			tdy = height / 2
			
		# X-Axis pointing to right. drawn in red
		x1 = size * (math.cos(yaw) * math.cos(roll)) + tdx
		y1 = size * (math.cos(pitch) * math.sin(roll) + math.cos(roll) * math.sin(pitch) * math.sin(yaw)) + tdy

		# Y-Axis | drawn in green
		#        v
		x2 = size * (-math.cos(yaw) * math.sin(roll)) + tdx
		y2 = size * (math.cos(pitch) * math.cos(roll) - math.sin(pitch) * math.sin(yaw) * math.sin(roll)) + tdy
		
		# Z-Axis (out of the screen) drawn in blue
		x3 = size * (math.sin(yaw)) + tdx
		y3 = size * (-math.cos(yaw) * math.sin(pitch)) + tdy
		
		cv2.line(img, (int(tdx), int(tdy)), (int(x1),int(y1)),(0,0,255),5)
		cv2.line(img, (int(tdx), int(tdy)), (int(x2),int(y2)),(0,255,0),5)
		cv2.line(img, (int(tdx), int(tdy)), (int(x3),int(y3)),(255,0,0),5)
		
		return img
        	
	def class_to_label(self, x):
		"""
		For a given label value, return corresponding string label.
		:param x: numeric label
		:return: corresponding string label
		"""
		
		return self.classes[int(x)]
	
	def callback(self,rgb_msg,camera_info,depth_msg,depth_info):
		rgb_image       = np.frombuffer(bytes(rgb_msg.data), np.uint8)
		rgb_image       = cv2.imdecode(rgb_image,cv2.IMREAD_COLOR)

		camera_info_K_l = np.array(camera_info.k).reshape([3, 3])
		camera_info_D   = np.array(camera_info.d)
		rgb_undist1     = cv2.undistort(rgb_image, camera_info_K_l, camera_info_D)
		
		depth_fmt, compr_type = depth_msg.format.split(';')
		
		# remove white space
		depth_fmt = depth_fmt.strip()
		compr_type = compr_type.strip()
		if compr_type != "compressedDepth":
			raise Exception("Compression type is not 'compressedDepth'."
                                        "You probably subscribed to the wrong topic.")
		
		# remove header from raw data
		depth_header_size = 12
		raw_data = depth_msg.data[depth_header_size:]
		buf = np.ndarray(shape=(1, len(raw_data)),
                          dtype=np.uint8, buffer=raw_data)
		
		depth_img_raw = cv2.imdecode(buf, cv2.IMREAD_ANYDEPTH | cv2.IMREAD_ANYCOLOR )
		if depth_img_raw is None:
		#	# probably wrong header size
			raise Exception("Could not decode compressed depth image."
                                        "You may need to change 'depth_header_size'!")


		results = self.score_frame(rgb_undist1)
		rgb_undist1 = self.plot_boxes(results,rgb_undist1, rgb_msg, camera_info, depth_img_raw,depth_info)
		
		if depth_fmt == "16UC1":
			img_scaled = cv2.normalize(depth_img_raw, dst=None, alpha=0, beta=65535, norm_type=cv2.NORM_MINMAX)
			cv2.imshow('Profundidade',img_scaled)
		cv2.imshow('Imagem',rgb_undist1)	
		if cv2.waitKey(1) & 0xFF == ord(' '):
      			print('asneira')
    
if __name__ == '__main__':
	global node
	
	ic = YoloROS()
	try:
		while rclpy.ok():
			rclpy.spin_once(node)
		
		node.destroy_node()
		rclpy.shutdown()
	
	except KeyboardInterrupt:
		print("Aborta")

