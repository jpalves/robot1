#!/usr/bin/env python3
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist
from simple_pid import PID
from geometry_msgs.msg import PoseStamped

import time
import math

g_node = None

pid_v = PID( .2, 0, 0.002, setpoint=0.0)
pid_z = PID( 0.4, 0, 0.001, setpoint=0.0)

start_time = time.time()
last_time = start_time

pid_v.output_limits =(-1.0,1.0)
pid_z.output_limits =(-1.0,1.0)

def callback(data):
     twist   = Twist()
     global start_time, last_time, navigator
     current_time = time.time()
     dt = current_time - last_time
     
     
     if math.fabs(data.linear.x) < 0.005:
        pid_v.reset()
        twist.linear.x  =  0.0 #-pid_v.__call__(0.0,dt)
     else:
        twist.linear.x  = -pid_v.__call__(data.linear.x,dt)
     
     
     
     if math.fabs(data.angular.z) < 0.003:
        pid_z.reset()
        twist.linear.z  =  0.0 #-pid_z.__call__(0.0,dt)
     else:
        twist.angular.z = -pid_z.__call__(data.angular.z,dt)

     rclpy.logging.get_logger('angular <-').info(str(twist.angular.z)+" [inear <-]: "+str(twist.linear.x))   
     pub.publish(twist)
     last_time = current_time
     

def main(args=None):
	global g_node
	global pub
	rclpy.init(args=args)
	
	g_node = rclpy.create_node('subscriber')
	pub = g_node.create_publisher(Twist, '/input', 10)
	subscription = g_node.create_subscription(Twist, '/cmd_vel', callback, 10)
	
	subscription  # prevent unused variable warning
	
	while rclpy.ok():	 
		rclpy.spin_once(g_node)
		

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	g_node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
    main()
