import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class user_inpt_publshr(Node):
    def __init__(self):
        super().__init__('user_input_publisher')
        self.publisher = self.create_publisher(String,'robot_arm_angles_here',10)
        self.get_logger().info("publshr ready to send msgs")
    
    def publsh_usr_inpt(self,usr_ip):
        msg = String()
        msg.data = usr_ip
        self.publisher.publish(msg)
        self.get_logger().info(f'Publshn {msg.data}')
    

def main(args=None):
    rclpy.init(args=args)
    nod = user_inpt_publshr()
    try:
        while True:
            usr_inpt = input('Entr a string to publish ') # please keep angle format as a010b030m070d120f130 whre no stands as angle values in dgrs
            nod.publsh_usr_inpt(usr_inpt)
    except KeyboardInterrupt:
        pass
    finally:
        nod.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()