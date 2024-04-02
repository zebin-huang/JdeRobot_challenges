import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class SimpleNavigationGoals(Node):
    def __init__(self):
        super().__init__('simple_navigation_goals')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.timer = self.create_timer(1, self.send_goal)

    def send_goal(self):
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = -2.0  # 硬编码的x坐标
        goal.pose.position.y = 0.0  # 硬编码的y坐标
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0  # 确保四元数是有效的，这里表示无旋转
        self.publisher.publish(goal)
        self.get_logger().info('Sending goal: (%.2f, %.2f)' % (goal.pose.position.x, goal.pose.position.y))

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavigationGoals()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

