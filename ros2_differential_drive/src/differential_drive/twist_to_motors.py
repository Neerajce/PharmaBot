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
        self.ard = serial.Serial("/dev/ttyUSB0",115200)
        self.w = self.declare_parameter("base_width", 0.45).value
        self.dx = 0.0
        self.dr = 0.0
        self.ticks_since_target = 0 #dis is useless
        self.velct_min = 0.05
        self.velct_max = 0.5
        self.pwm_min = 40
        self.pwm_max = 255
        self.pwm = 0

        self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)

        self.rate_hz = self.declare_parameter("rate_hz", 50).value
        
        self.create_timer(1.0, self.calculate_left_and_right_target_and_give_values)

    def velct_to_pwm(self,vel):

        self.pwm = self.pwm_min + ((vel - self.velct_min)/(self.velct_max - self.velct_min)) * (self.pwm_max - self.pwm_min)

        return self.pwm
    

    def calculate_left_and_right_target_and_give_values(self):
        # dx = (l + r) / 2
        # dr = (r - l) / w

        right = Float32()
        left = Float32()
        
        right.data = self.dx

        right.data = 1.0 * self.dx + self.dr * self.w / 2.0
        left.data = 1.0 * self.dx - self.dr * self.w / 2.0

        print('right.data vel is ',right.data)
        print('left.data vel is ',left.data)
        intrplt_right_data = int(self.velct_to_pwm(right.data))
        intrplt_left_data = int(self.velct_to_pwm(left.data))
        print('intrplt_right_data is ',intrplt_right_data)
        print('intrplt_left_data is ',intrplt_left_data)

        right_data_to_Strng = str(intrplt_right_data)
        left_data_to_Strng = str(intrplt_left_data)

        left_and_rght_data_to_strng = left_data_to_Strng + ','+ right_data_to_Strng + ' '
        print('left_and_rght_data_to_strng is ',left_and_rght_data_to_strng)
        self.ard.write(left_and_rght_data_to_strng.encode())
        
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
