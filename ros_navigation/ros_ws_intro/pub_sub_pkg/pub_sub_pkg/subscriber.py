import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MySubscriber(Node):
    def __init__(self):
        super().__init__('fun_subscriber')
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)
      	# print(self.subscription)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    my_subscriber = MySubscriber()
    rclpy.spin(my_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
