#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration
from robot_navigator import BasicNavigator, NavigationResult # Helper module
import math
import db
import numpy as np

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

     #sinr_cosp = 2 * (w * x + y * z)
     #cosr_cosp = 1 - 2 * (x * x + y * y)
     #roll = np.arctan2(sinr_cosp, cosr_cosp)

     #sinp = 2 * (w * y - z * x)
     #pitch = np.arcsin(sinp)

     siny_cosp = 2 * (w * z + x * y)
     cosy_cosp = 1 - 2 * (y * y + z * z)
     yaw = np.arctan2(siny_cosp, cosy_cosp)

     return yaw
    
#sudo apt install ros-galactic-tf-transformations
class MinimalSubscriber(Node):
    
    def __init__(self):
        super().__init__('dock')
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
        quat = self.q.quaternion_from_euler(0, 0, 0)
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_pose.pose.position.z = 0.0
        self.goal_pose.pose.orientation.x =  quat.x
        self.goal_pose.pose.orientation.y =  quat.y
        self.goal_pose.pose.orientation.z =  quat.z
        self.goal_pose.pose.orientation.w =  quat.w
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag_detections',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        rclpy.spin(self)
        
        
    def listener_callback(self, msg):
    	
    	# Set the robot's goal pose
    	
    	for m in msg.detections:
    		if m.id == 0 and self.once:
    			try:
    				t = self.tf_buffer.lookup_transform("map", "tag_0", rclpy.time.Time(),timeout=rclpy.duration.Duration(seconds=1))
    			except:
    				break
    			once = False 
    			self.machines.inserir(m.id,t.transform.translation.x, t.transform.translation.y,0,'tag_0')
    			self.goal_pose.pose.position.x =  t.transform.translation.x + 0.7
    			self.goal_pose.pose.position.y =  t.transform.translation.y + 0.2
    			
    			self.navigator.goToPose(self.goal_pose)
    			i = 0
    			print(t.transform.translation.y)
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
    			if result == NavigationResult.SUCCEEDED:
    			 	print('Goal succeeded!')
    			 	exit(0)
    		else:
    			try:
    				t = self.tf_buffer.lookup_transform("map", "tag_" + str(m.id), rclpy.time.Time(),timeout=rclpy.duration.Duration(seconds=1))
    			except:
    				break
    				
    			q = Quaternion()
    			q.x = t.transform.rotation.x
    			q.y = t.transform.rotation.y
    			q.z = t.transform.rotation.z
    			q.w = t.transform.rotation.w
    			self.machines.inserir(m.id,t.transform.translation.x ,t.transform.translation.y,q.yaw_from_quaternion(q),"tag_"+str(m.id))
					
    	#self.get_logger().info('em testes')

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
