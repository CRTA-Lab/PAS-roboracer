import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Header
import numpy as np
import math



class WallFollow(Node):
    def __init__(self):
        super().__init__("wall_follow_node")
        self.get_logger().info("This node just says 'Hello'")
        self.scan_sub = self.create_subscription(LaserScan, "scan", self.callback_scan, 10)
        self.publisher_ = self.create_publisher(AckermannDriveStamped, "drive", 10)
        self.vehicle_speed_subscriber = self.create_subscription(AckermannDrive, "vehicle_speed", self.vehicle_speed_callback, 10)


        self.angle_min = -135
        self.angle_increment = 0.25
        self.declare_parameter('vehicle_speed1', 1.4)
        self.declare_parameter('vehicle_speed2', 1.4)
        self.declare_parameter('vehicle_speed3', 1.4)
        self.declare_parameter('look_ahead', 1.0)
        self.declare_parameter('wall_dist', 0.7)

        self.Kp = 1.0
        self.Kd = 0.001
        self.Ki =0.005
        self.prev_error = 0.0
        self.prev_time = None
    
        self.vehicle_speed = 1.0
        

        # TODO: store history
        self.integral = 0.0
        
        self.error = 0.0
        self.speed = 0.0
                
    def vehicle_speed_callback(self,msg):
        self.vehicle_speed = msg.speed

        
    def callback_scan(self,msg):
        self.ranges = msg.ranges
        self.error = self.get_error(msg.ranges, self.get_parameter('wall_dist').get_parameter_value().double_value)     
        self.pid_control(self.error)
        

    def get_range(self,range_data, angle):
        self.index = round((angle - self.angle_min) / self.angle_increment)
        return range_data[self.index]
        
    def get_error(self,range_data,dist):
        a_ = range_data[720]       
        b_ = range_data[900]       


        self.alfa = np.arctan((a_ * np.cos(math.pi/4)- b_)/(a_ * np.sin(math.pi/4))) 


        currDt = b_ * math.cos(self.alfa)

        l = self.get_parameter('look_ahead').get_parameter_value().double_value
        act_dist = currDt + l * np.sin(self.alfa)
        
        self.error = dist - act_dist

        return self.error
    
    def pid_control(self, error):
        
        self.curr_time = self.get_clock().now()
        
        if self.prev_time is not None:
            time__  = self.curr_time.nanoseconds - self.prev_time.nanoseconds
            time_ = time__ / 1e9
            proportional = self.Kp * error
            self.integral = self.integral + self.Ki * time_ * error
            if self.integral > 20:
                self.integral = 0.0
            derivative = self.Kd*(error - self.prev_error)/ time_

            self.angle = - proportional - self.integral - derivative

            

            if (self.angle >= 0.0) and (self.angle < 10.0):
               
                self.speed = self.vehicle_speed 
            elif (self.angle > 10.0) and (self.angle < 20.0):
                
                self.speed = self.vehicle_speed
            elif (self.angle < 0.0) and (self.angle > -1.0):
                
                self.speed = self.vehicle_speed
            else:
                
                self.speed = self.vehicle_speed
            


            msg = AckermannDriveStamped()

            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()  
            msg.header.frame_id = 'base_link'

            drive_msg = AckermannDrive()
            drive_msg.steering_angle = self.angle
            drive_msg.speed = self.speed 
            msg.drive = drive_msg

            self.publisher_.publish(msg)

        self.prev_error = error 
        self.prev_time = self.curr_time

    def stop(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        for _ in range(3):
            self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = WallFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C: stopping car")
        node.stop()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()
