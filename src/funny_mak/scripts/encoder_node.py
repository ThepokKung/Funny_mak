#!/usr/bin/python3

import rclpy
from rclpy.node import Node
#My import
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class EncoderNode(Node):
    def __init__(self):
        super().__init__('Encoder_Node')
        #Sub Clinet
        self.create_subscription(Twist,'motor_speed',self.MotorSpeed_Callback,10) #motor speed sub
        #Pub Clinet
        self.motor_postiion_pub = self.create_publisher(Float64,'motor_position',10) #Motor Position Pub
        #Val 
        self.angular_z_speed = 0 #Init Z speed for make val
        self.position_z = 0 #Init Z position
        self.z_position_temp = 0
        #Timer
        self.create_timer(0.01,self.Timer_Callback) #Timer 
        #Get logger
        self.get_logger().info("")
        
    #Motor_Speed Callback
    def MotorSpeed_Callback(self,msg):
        self.angular_z_speed = msg.angular.z
        self.Angular_Cal()

    #Angular_Calculator
    def Angular_Cal(self):
        self.z_position_temp += (self.angular_z_speed * (0.01)) *(360/(2*math.pi)) # delta_theta(deg) = (w(rad) * delta_t) * (360/(2*pi))
        self.position_z  = self.z_position_temp % (360) # Warp 360
        # print(self.position_z)
    
    def Timer_Callback(self):
        self.Angular_Cal()
        self.postion_pub(self.position_z)
        # self.get_logger().info(f"Motor position Pub : {self.position_z} degrees")

    def postion_pub(self,data):
        msg = Float64()
        msg.data = data
        self.motor_postiion_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
