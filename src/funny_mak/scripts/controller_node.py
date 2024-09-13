#!/usr/bin/python3

import rclpy
from rclpy.node import Node

#My Import
from std_msgs.msg import Float64,Int8

# Interfaces
from fun35_controller_interfaces.srv import SetKParam

# State Note
# 4 is reafy
# 5 is wait to match (Working) 
# 6 is wait for next

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        #Sub
        self.create_subscription(Float64,'motor_position',self.Motor_position_callback,10) #Motor Postion from encoder  
        self.create_subscription(Float64,'/target',self.Target_Callback,10) #Target from Scheduler
        self.create_subscription(Int8,'/notify',self.Notify_Callback,10) # Notify Sub

        #Pub
        self.control_signal_pub = self.create_publisher(Float64,'control_signal',10) #Pub Volte to dc_motorsim
        self.notify_pub = self.create_publisher(Int8,'/notify',10) #Notify Pub

        #Timer
        self.create_timer(0.01,self.Timer_callback) #Timer

        #Serive 
        self.set_param_server = self.create_service(SetKParam,'k_param',self.set_kParam_callback) #Sup config servieAnd Use Realative topic don't fix it

        #Val
        self.motor_postion = 0
        self.target_postion = 0
        self.voltage = 0
        self.state = 0

        #PID control
        self.prev_error = 0
        self.integral = 0
        self.Kp = 0.065  # Proportional gain
        self.Ki = 0.00065  # Integral gain
        self.Kd = 0.0  # Derivative gain

        # Time variables for derivative calculation
        self.last_time = self.get_clock().now()

    #Timer Callback
    def Timer_callback(self):
        self.get_logger().info(f'My State is : {self.state}')

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

        # Check Math postion
        if self.state == 5 and abs(error_position) < 0.1:
            self.state = 6

    #Motor Postion Callback
    def Motor_position_callback(self,msg):
        self.motor_postion = msg.data
        self.get_logger().info(f"Motor position: {self.motor_postion} degrees")

    #Notify Callback
    def Notify_Callback(self,msg):
        state = msg.data
        if state == 1 and self.state == 0:
            #Check controller Ready
            self.Send_heartbeat(4)
            self.state = 4 #Ready
            self.get_logger().info(f'My State is : {self.state}')

        elif state == 2 and self.state == 6:
            #Wait for target
            self.Send_heartbeat(6)
            self.state = 4
        else:
            pass
        
    def Send_heartbeat(self,state_data):
        msg = Int8()
        msg.data = state_data
        self.notify_pub.publish(msg)
        self.get_logger().info(f'Publised notify state : {state_data}')

    #Target Callback 
    def Target_Callback(self,msg):
        if self.state == 4:
            self.target_postion = msg.data
            # print(self.target_postion)
            self.get_logger().info(f"Target position: {self.target_postion} degrees")
            self.state = 5

    #Send Voltage dato for control motor
    def signal_pub(self,data):
        msg = Float64()
        msg.data = data
        self.control_signal_pub.publish(msg)
        # self.get_logger().info(f"Control signal (voltage): {data:.2f} voltage")

    def set_kParam_callback(self,request:SetKParam.Request,response:SetKParam.Response):
        print(request)
        if request.kp.data != 0.0:
            self.Kp = request.kp.data
        if request.ki.data != 0.0:
            self.Ki = request.ki.data
        if request.kd.data != 0.0:
            self.Kd = request.kd.data
        self.get_logger().info(f'Set Kp : {self.Kp}, Ki : {self.Ki}, Kd : {self.Kd}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
