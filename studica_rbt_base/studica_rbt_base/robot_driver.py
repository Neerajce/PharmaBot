#!/usr/bin/env python3
import serial
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
import tf2_ros
import tf_transformations
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
import math

class Encoder_publishr_subscrbr(Node):
    def __init__(self):
        super().__init__('encoder_publshr_subscrbr')
        self.ard = serial.Serial("/dev/ttyUSB0",9600)
        #self.publishr = self.create_publisher(Int16,'encoder_ticks',10) isn't used
        self.publshr_odom = self.create_publisher(Odometry,'odom',10)
        self.get_logger().info('Encoder Publisher Node has been started.')
        self.timer = self.create_timer(1.0,self.publish_odomtry)
        self.pwm_motor_right_old = 0
        self.pwm_motor_left_old = 0
        self.ticks_per_revolutn_left_wheel = 367
        self.ticks_per_revolutn_right_wheel = 367
        # self.radius_left_wheel = 6.2 dis isn't used curntly
        # self.radius_right_wheel = 6.2
        self.distnc_lft = 0
        self.distnc_rght = 0
        self.disnc_trvld_by_cntr_pt_of_rbt = 0
        self.robot_turn_angle = 0
        self.wheel_base = 0.45 # cntr of left tire to cntr of rght tire


    # def write_ser_helpr(self,cmd): useful for giving input to arduino but not useful curntly here.
    #     self.ard.write(cmd)
    def update_odom(self):
         self.disnc_trvld_by_cntr_pt_of_rbt = (self.distnc_lft + self.distnc_rght)/2
         self.robot_turn_angle = math.asin((self.distnc_rght - self.distnc_lft)/self.wheel_base)
         if self.robot_turn_angle > 3.14:
            self.robot_turn_angle -=2*3.14
         elif self.robot_turn_angle < -3.14:
              self.robot_turn_angle +=2*3.14
            





    def publish_odomtry(self):
        line = self.ard.readline().decode("utf-8").strip()
        temp  = line.split(',')
        print(temp)
        pwm_motor_right_fresh = int(temp[0])
        pwm_motor_left_fresh = int(temp[1])
        
        pwm_motor_right_diff = pwm_motor_right_fresh - self.pwm_motor_right_old
        pwm_motor_left_diff = pwm_motor_left_fresh - self.pwm_motor_left_old
        
        self.pwm_motor_right_old = pwm_motor_right_fresh
        self.pwm_motor_left_old = pwm_motor_left_fresh
        
        self.distnc_lft = pwm_motor_left_diff/ self.ticks_per_revolutn_left_wheel
        self.distnc_right = pwm_motor_right_diff / self.ticks_per_revolutn_right_wheel
    
        self.update_odom(self)
    
        self.odom_dta = Odometry()
        self.odom_dta.header.stamp = self.get_clock().now().to_msg()
        self.odom_dta.header.frame_id = 'odom'
        self.odom_dta.child_frame_id = 'base_link'
    
def main(args = None):
    rclpy.init(args=args)
    node =  Encoder_publishr_subscrbr()
    try:
            rclpy.spin(node)
    except KeyboardInterrupt:
         pass
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
     main()





