#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration
from robot_navigator import BasicNavigator, NavigationResult # Helper module
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

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
        super().__init__('dock_broadcast')
        self.tf_buffer = Buffer()
        self.tf = TransformListener(self.tf_buffer,self)
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.goal_pose = PoseStamped()
        self.q = Quaternion()
        self.machines = db.Db()
        #print(self.r)
        self.machines.criar_schema()
        self.once = True
        self.r = self.machines.ler()
        self.tolx = [0]*len(self.r)
        self.toly = [0]*len(self.r)
        self.tolx[0]  = 0.95
        self.toly[0]  = 0.2
        self.tolx[1]  = 0 
        self.tolx[27] = 0
        self.toly[27] = 0
        self._tf_publisher = StaticTransformBroadcaster(self)
        # Publish static transforms once at startup
        self._tf_publisher.sendTransform(self.listener_callback())
        rclpy.spin(self)
        


    def listener_callback(self):
        out = ()

        for i in range(0,len(self.r)):
            self.get_logger().info(str(self.r[i]))
            aux = TransformStamped()
            aux.header.frame_id = 'map'
            aux.header.stamp = self.get_clock().now().to_msg()
            aux.child_frame_id = self.r[i][4]
            aux.transform.translation.x =  self.r[i][1] + self.tolx[self.r[i][0]]
            aux.transform.translation.y =  self.r[i][2] + self.toly[self.r[i][0]]
            aux.transform.translation.z = 0.0
    		
            q  = Quaternion()
            qq = q.quaternion_from_euler(0, 0, self.r[i][3])
            
            aux.transform.rotation.w = qq.w
            aux.transform.rotation.x = qq.x
            aux.transform.rotation.y = qq.y
            aux.transform.rotation.z = qq.z
            
            temp = list(out)
            temp.append(aux)
            out  = tuple(temp)
        
        return out 

def main(args=None):
    rclpy.init(args=args)
    try:
        minimal_subscriber = MinimalSubscriber()
    
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        minimal_subscriber.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print('Abortado')
    
if __name__ == '__main__':
    main()
