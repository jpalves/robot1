#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration
from robot_navigator import BasicNavigator, NavigationResult # Helper module
from std_msgs.msg import Int32

from enum import Enum

import math
import db
import numpy as np
import time

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
      
    def yaw_from_quaternion(self):
     siny_cosp = 2 * (self.w * self.z + self.x * self.y)
     cosy_cosp = 1 - 2 * (self.y * self.y + self.z * self.z)
     yaw = np.arctan2(siny_cosp, cosy_cosp)

     return yaw


#sudo apt install ros-galactic-tf-transformations

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('machines')
        self.tf_buffer = Buffer()
        self.tf = TransformListener(self.tf_buffer,self)
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.goal_pose = PoseStamped()
        self.q = Quaternion()
        self.machines = db.Db()
        self.r = self.machines.ler()
        print(self.r)
        self.machines.criar_schema()
        self.once = True
        #quat = self.q.quaternion_from_euler(0, 0, -1.57)
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_pose.pose.position.z = 0.0
        self.machine = []
        self.tempo   = 0
        self.publisher_ = self.create_publisher(Int32, 'fim', 10)
        
        
        
        self.timer = self.create_timer(
            0.001, lambda: self.timer_callback()) 
        self.subscription = self.create_subscription(
            Int32,
            '/machine',
            self.listener_callback,
            100)
            
        self.subscription
        rclpy.spin(self)
        
    def listener_callback(self,msg):
    	self.machine.append(msg.data)
    	self.once = True
    	
    def timer_callback(self):
    	if self.once:
    		start = time.time()
    		try:
    			t = self.tf_buffer.lookup_transform("map","tag_"+str(self.machine[0]), rclpy.time.Time(),timeout=rclpy.duration.Duration(seconds=1))
    		except:
    			return	
    		self.once = False
    		self.goal_pose.pose.position.x =  t.transform.translation.x
    		q = Quaternion()
    		q.x = t.transform.rotation.x
    		q.y = t.transform.rotation.y
    		q.z = t.transform.rotation.z
    		q.w = t.transform.rotation.w
    	
    		if math.fabs(q.yaw_from_quaternion()) > 0.08:
    			self.goal_pose.pose.position.y =  t.transform.translation.y  + 0.6
    			quat = self.q.quaternion_from_euler(0, 0, -math.pi/2.0)
    		elif math.fabs(q.yaw_from_quaternion()) == 0.0:
    			self.goal_pose.pose.position.x =  t.transform.translation.x
    			self.goal_pose.pose.position.y =  t.transform.translation.y
    			quat = self.q.quaternion_from_euler(0, 0, 0)
    		else:
    			self.goal_pose.pose.position.y =  t.transform.translation.y - 0.6
    			quat = self.q.quaternion_from_euler(0, 0,  math.pi/2.0)
    		
    		self.goal_pose.pose.orientation.x =  quat.x
    		self.goal_pose.pose.orientation.y =  quat.y
    		self.goal_pose.pose.orientation.z =  quat.z
    		self.goal_pose.pose.orientation.w =  quat.w
    			
    		self.navigator.goToPose(self.goal_pose)
    		i = 0
    		while not self.navigator.isNavComplete():
    			i = i + 1
    			feedback = self.navigator.getFeedback()
    			if feedback and i % 5 == 0:
    				print('Distancia em falta: ' + '{:.2f}'.format(feedback.distance_remaining) + ' metros.')
    			
    			if Duration.from_msg(feedback.navigation_time) > Duration(seconds=120.0):
    				self.goal_pose.pose.position.x =   1.00
    				self.goal_pose.pose.position.y =   0.0
    				self.navigator.goToPose(self.goal_pose)
    			
    		result = self.navigator.getResult()
    		if result == NavigationResult.SUCCEEDED or result == NavigationResult.FAILED:
    			msg = Int32()
    			msg.data = self.machine[0]
    			self.publisher_.publish(msg)
    			self.navigator.clearAllCostmaps()
    			self.tempo += time.time() - start
    			rclpy.logging.get_logger('Máquina: ').info(str(self.machine[0]))
    			self.machine.pop(0)
    			time.sleep(10)
    			rclpy.logging.get_logger('Tempo: ').info(str(self.tempo)+ "s")
    			if len(self.machine) == 0:
    				self.tempo = 0
    			else:
    				self.once = True
    			
    	
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    
    
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
