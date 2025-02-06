import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
import numpy as np
from tf2_ros.transform_listener import TransformListener
import math
 
class user_inpt_publshr(Node):
    def __init__(self):
        super().__init__('user_input_publisher')
        self.publisher = self.create_publisher(String,'robot_arm_angles_here',10)
        self.get_logger().info("publshr ready to send msgs")
        self.from_frame_rel = 'tag36h11:0'
        self.to_frame_rel = 'camera_link'
        self.timer  = self.create_timer(0.001,self.on_timer)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)
        self.l1 = 100
        self.l2 = 200
        
    # def euler_from_quaternion(self, x, y, z, w):
    #         """
    #         Convert a quaternion into euler angles (roll, pitch, yaw)
    #         roll is rotation around x in radians (counterclockwise)
    #         pitch is rotation around y in radians (counterclockwise)
    #         yaw is rotation around z in radians (counterclockwise)
    #         """
    #         t0 = +2.0 * (w * x + y * z)
    #         t1 = +1.0 - 2.0 * (x * x + y * y)
    #         roll_x = math.atan2(t0, t1)
        
    #         t2 = +2.0 * (w * y - z * x)
    #         t2 = +1.0 if t2 > +1.0 else t2
    #         t2 = -1.0 if t2 < -1.0 else t2
    #         pitch_y = math.asin(t2)
        
    #         t3 = +2.0 * (w * z + x * y)
    #         t4 = +1.0 - 2.0 * (y * y + z * z)
    #         yaw_z = math.atan2(t3, t4)
        
    #         return [roll_x, pitch_y, yaw_z] # in radians

    def publsh_usr_inpt(self,usr_ip):

        x_val = 90
        z_val = 90
        num = x_val * x_val + z_val * z_val - self.l1 * self.l1 - self.l2 * self.l2
        den = 2*self.l1*self.l2
        theta2  = np.arccos(num/den)
        theta1 = np.arctan((self.l2*np.sin(theta2))/(self.l1 + self.l2 * np.cos(theta2)))

        msg = String()
        msg.data = str(theta1) + ',' + str(theta2) + ','
        self.publisher.publish(msg)
        self.get_logger().info(f'Publshn {msg.data}')
    
    def on_timer(self):
        try:
            #self.tf_buffer.can_transform(self.to_frame_rel, self.from_frame_rel, rclpy.time.Time(), timeout=1.0)
        
            t = self.tf_buffer.lookup_transform('robot_arm_bl','tag36h11:0',rclpy.time.Time(),timeout=rclpy.duration.Duration(seconds=1.0))
        except TransformException as ex:
            self.get_logger().info('could not transform')
            return
        print('_1 below_')
        print(t.transform.translation.x)
        print('_2 below_')
        print(t.transform.translation.y)
        print('_3 below_')
        print(t.transform.translation.z)

        #print(self.euler_from_quaternion(t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w))
        

    

def main(args=None):
    rclpy.init(args=args)
    nod = user_inpt_publshr()
    try:
        while True:
            rclpy.spin(nod)
            #nod.on_timer()
            #usr_inpt = input('Entr a string to publish ') # please keep angle format as f 010020030040050v whre nos stands as angle values in dgrs , here 010, 020 stands for 10 and 20 dgrs of waist and shouldr rspctvly
            #nod.publsh_usr_inpt(usr_inpt)
    except KeyboardInterrupt:
        pass
    finally:
        nod.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()