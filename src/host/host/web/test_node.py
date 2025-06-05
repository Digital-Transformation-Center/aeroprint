import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, String
import json

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.status_publisher = self.create_publisher(Int8, 'drone_status', 10)
        self.param_subscriber = self.create_subscription(String, 'helix_params', self.param_callback, 10)
         # Shutdown subscriber
        self.shutdown_sub = self.create_subscription(Int8, 'test_node_shutdown', self.shutdown_callback, 10)

        # Heartbeat publisher
        from rclpy.qos import QoSProfile
        self.heartbeat_publisher = self.create_publisher(Int8, 'heartbeat', 10)
        self.create_timer(0.1, self.publish_heartbeat)

    def shutdown_callback(self, msg):
        if msg.data == 99:
            self.get_logger().info('Shutdown signal received. Shutting down test_node.')
            raise KeyboardInterrupt("Shutdown signal received, exiting.")

    def publish_heartbeat(self):
        msg = Int8()
        msg.data = 1  # 1 means alive
        self.heartbeat_publisher.publish(msg)

    def param_callback(self, msg):
        self.get_logger().info(f"Received params: {msg.data}")
        try:
            data = json.loads(msg.data)
            radius = data['radius']
            height = data['height']
            turns = data['turns']
            start_height = data['startHeight']

            self.get_logger().info(f"Drone set params: radius{radius}")
            self.publish_param_success()
        except Exception :
            self.get_logger().error("Unable to parse json data.")
            self.publish_param_fail()

    def publish_param_success(self):
        msg = Int8()
        msg.data = 3
        self.status_publisher.publish(msg)

    def publish_param_fail(self):
        msg = Int8()
        msg.data = 4
        self.status_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()