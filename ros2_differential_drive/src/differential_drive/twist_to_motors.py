#!/usr/bin/env python

# Copyright (C) 2012 Jon Stephan.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import serial


class TwistToMotors(Node):
    """
    twist_to_motors - converts a twist message to motor commands.  Needed for navigation stack

    """

    def __init__(self):
        super(TwistToMotors, self).__init__("twist_to_motors")
        self.nodename = "twist_to_motors"
        self.get_logger().info("%s started" % self.nodename)
        self.ard = serial.Serial("/dev/ttyUSB0",9600)
        self.w = self.declare_parameter("base_width", 0.45).value
        self.dx = 0
        self.dr = 0
        self.ticks_since_target = 0

        self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)

        self.rate_hz = self.declare_parameter("rate_hz", 50).value
        
        self.create_timer(1.0/self.rate_hz, self.calculate_left_and_right_target_and_give_values)

    def write_ser_helpr(self,cmd): #useful for giving input to arduino but not useful curntly here.
        self.ard.write(cmd.encode())

    def calculate_left_and_right_target_and_give_values(self):
        # dx = (l + r) / 2
        # dr = (r - l) / w

        right = Float32()
        left = Float32()
        
        right.data = 1.0 * self.dx + self.dr * self.w / 2.0
        left.data = 1.0 * self.dx - self.dr * self.w / 2.0
        right_data_to_Strng = str(right.data)
        left_data_to_Strng = str(left.data)

        left_and_rght_data_to_strng = left_data_to_Strng + right_data_to_Strng

        self.write_ser_helpr(left_and_rght_data_to_strng)
        
        self.ticks_since_target += 1

    def twist_callback(self, msg):
        self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dr = msg.angular.z


def main(args=None):
    rclpy.init(args=args)
    try:
        twist_to_motors = TwistToMotors()
        rclpy.spin(twist_to_motors)
    except rclpy.exceptions.ROSInterruptException:
        pass

    twist_to_motors.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
