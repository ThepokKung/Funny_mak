#!/usr/bin/python3

import rclpy
from rclpy.node import Node

#My Import
from std_msgs.msg import Float64,Int8

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        #Sub
        self.create_subscription(Float64,'/motor6504/motor_position',self.Motor_position_callback,10) #Motor Postion from encoder  
        self.create_subscription(Float64,'/target',self.Target_Callback,10) #Target from Scheduler
        self.create_subscription(Int8,'/notify',self.Notify_Callback,10) # Notify Sub

        #Pub
        self.control_signal_pub = self.create_publisher(Float64,'/motor6504/control_signal',10) #Pub Volte to dc_motorsim
        self.notify_pub = self.create_publisher(Int8,'/notify',10) #Notify Pub

        #Timer
        self.create_timer(0.01,self.Timer_callback) #Timer

        #Val
        self.motor_postion = 0
        self.target_postion = 0
        self.Notify = 1
        self.voltage = 0

        #PID control
        self.prev_error = 0
        self.integral = 0
        self.Kp = 0.065  # Proportional gain
        self.Ki = 0.00055  # Integral gain
        self.Kd = 0.0  # Derivative gain

        # Time variables for derivative calculation
        self.last_time = self.get_clock().now()

    #Timer Callback
    def Timer_callback(self):
        error_position = self.target_postion - self.motor_postion

        #Condition for fine near postion
        if error_position > 180:
            error_position = error_position - 360
        elif error_position < -180:
            error_position = error_position + 360

        # PID control calculations
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert time difference to seconds

        if dt > 0:  # Avoid division by zero
            # Proportional term
            proportional = self.Kp * error_position

            # Integral term
            self.integral += error_position * dt
            integral = self.Ki * self.integral

            # Derivative term
            derivative = self.Kd * (error_position - self.prev_error) / dt

            # PID output (control signal)
            control_signal = proportional + integral + derivative

            # Apply control signal (output voltage)
            self.signal_pub(control_signal)

            # Update previous error and time
            self.prev_error = error_position
            self.last_time = current_time

        #Condition for flip vector
        if error_position < 0:
            self.voltage * -1
        pass

    #Motor Postion Callback
    def Motor_position_callback(self,msg):
        self.motor_postion = msg.data
        # print(self.motor_postion)
        self.get_logger().info(f"Motor position: {self.motor_postion} degrees")

    #Notify Callback
    def Notify_Callback(self,msg):
        pass

    #Target Callback 
    def Target_Callback(self,msg):
        self.target_postion = msg.data
        # print(self.target_postion)
        self.get_logger().info(f"Target position: {self.target_postion} degrees")

    def signal_pub(self,data):
        msg = Float64()
        msg.data = data
        self.control_signal_pub.publish(msg)
        self.get_logger().info(f"Control signal (voltage): {data:.2f} voltage")

        
def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
