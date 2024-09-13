#!/usr/bin/python3

import rclpy
from rclpy.node import Node

import yaml,time,random,os
from std_msgs.msg import Int8

class RandomTargetNode(Node):
    def __init__(self):
        super().__init__('randomtarget_node')

        # Publisher to send heartbeat
        self.notify_pub = self.create_publisher(Int8, '/notify', 10)

        # Subscriber to listen for responses
        self.create_subscription(Int8, '/notify', self.Notify_Callback, 10)

        # Initial state
        self.state = 0  # Initial state: Waiting to send the first heartbeat

        # Parameters for random target generation
        self.declare_parameter('num_targets', 5)  # Number of random targets
        self.declare_parameter('target_min', 0)  # Min target (use int)
        self.declare_parameter('target_max', 360)  # Max target (use int)
        self.declare_parameter('file_yaml_path','~/fun3.5_ws/src/funny_mak/config/via_point.yaml')

        # Get parameter values
        self.num_targets = self.get_parameter('num_targets').value
        self.target_min = self.get_parameter('target_min').value
        self.target_max = self.get_parameter('target_max').value
        self.file_yaml_path = self.get_parameter('file_yaml_path').get_parameter_value().string_value

        # Generate random targets and save to YAML
        self.targets = self.generate_random_targets()
        self.save_to_yaml(self.targets)

        # Log
        self.get_logger().info(f"Generated {self.num_targets} random targets and saved to via_point.yaml")

    def generate_random_targets(self):
        # Generate a list of random integer target positions
        targets = [random.randint(self.target_min, self.target_max) for _ in range(self.num_targets)]
        return targets

    def save_to_yaml(self, targets):
        # Define the path to the YAML file
        yaml_file_path = os.path.expanduser(self.file_yaml_path)

        # Structure to save in YAML format
        data = {'targets': targets}

        # Write to the YAML file
        try:
            with open(yaml_file_path, 'w') as file:
                yaml.dump(data, file)
            self.get_logger().info(f"Targets saved to {yaml_file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save targets to YAML: {e}")

    def send_heartbeat(self):
        msg = Int8()
        msg.data = 9
        self.notify_pub.publish(msg)
        self.get_logger().info('Sent heartbeat: 9')

    def Notify_Callback(self, msg):
        # Check for the value 10 to stop the node
        if msg.data == 10:
            self.get_logger().info('Received 10, ready to shut down node.')
            self.state = 1  # Change the state to 1 to stop sending heartbeats

def main(args=None):
    rclpy.init(args=args)
    node = RandomTargetNode()

    try:
        while node.state != 1:  # Continue until the state is set to 1
            node.send_heartbeat()
            rclpy.spin_once(node, timeout_sec=1.0)  # Allow message processing
            time.sleep(1)  # Sleep for 1 second between heartbeats

        # Node will destroy itself after receiving "10"
        node.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted, shutting down.')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
