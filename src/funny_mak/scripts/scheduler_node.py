#!/usr/bin/python3

import rclpy
from rclpy.node import Node

import yaml
import os

from std_msgs.msg import Float64,Int8
from ament_index_python.packages import get_package_share_directory

# State Note
# 1 is ready
# 2 is Send Target
# 3 is Out of target

class SchedulerNode(Node):
    def __init__(self):
        super().__init__('scheduler_node')
        #Sub
        self.create_subscription(Int8,'/notify',self.Notify_Callback,10) # Notify Sub
        #Pub clien
        self.target_pub_client = self.create_publisher(Float64,'/target',10) #Pub Target
        self.notify_pub = self.create_publisher(Int8,'/notify',10) #Notify Pub
        #Timer 
        self.timer = self.create_timer(0.1, self.Timer_callback)

        # Parameters for random target generation
        self.declare_parameter('file_yaml_path','~/Funny_mak/src/funny_mak/config/via_point.yaml')
        
        # Get Param
        self.file_yaml_path = self.get_parameter('file_yaml_path').get_parameter_value().string_value

        #Read from yaml
        self.targets = self.Read_target_from_yaml()
        #VAL for Target
        self.target_index = 0 #Target index\
        self.state = 0 #Stage for notify

    def Timer_callback(self):
        #If data out of range 
        self.get_logger().info(f'My State is : {self.state}')
        if (self.target_index) >= len(self.targets):
            self.state = 3
            self.get_logger().info("Emepy data for send")
            return
        
        #State machine
        if self.state == 1:
            #Check Controller is ready
            self.Send_heartbeat(1)
        elif self.state == 2 and self.target_index < len(self.targets): #Check Tart out of range 
            #Send target
            target = self.targets[self.target_index]
            self.Target_pub(target)
            self.Send_heartbeat(2)

    def Target_pub(self,target_pose):
        msg = Float64()
        msg.data = float(target_pose)
        self.target_pub_client.publish(msg)
        self.get_logger().info(f'Published target postition : {target_pose}')

    def Notify_Callback(self,msg):
        state = msg.data
        #Switch case 
        if state == 9 and self.state == 0:
            self.state = 1
            self.targets = self.Read_target_from_yaml()
            self.Send_heartbeat(10)
        elif state == 4 and self.state == 1:
            self.state = 2
        elif state == 6 and self.state == 2:
            self.target_index += 1

            if self.target_index >= len(self.targets):
                self.state = 3 # Out of target
            else:
                self.state = 2
        else:
            pass

    def Send_heartbeat(self, value):
        msg = Int8()
        msg.data = value
        self.notify_pub.publish(msg)
        
    def Read_target_from_yaml(self):
       # Define the path to the YAML file
        yaml_file_path = os.path.expanduser(self.file_yaml_path)

        # Read the YAML file
        try:
            with open(yaml_file_path, 'r') as file:
                targets_data = yaml.safe_load(file)
                print(targets_data)
                return targets_data['targets']
        except Exception as e:
            self.get_logger().error(f"Failed to load targets from YAML: {e}")
            return []
        
def main(args=None):
    rclpy.init(args=args)
    node = SchedulerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
