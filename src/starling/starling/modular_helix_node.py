from starling.flightdefs.flightnode import FlightNode
from starling.flightdefs.helix import Helix
from std_msgs.msg import Float32MultiArray, Bool
import rclpy

class HelixNode(FlightNode):
    def __init__(self):
        name = "helical_flight_node"
        super().__init__(name)

        self.param_subscriber = self.create_subscription(
            Float32MultiArray, '/helix_params', self.helix_params_callback, 10
        )

        self.start_flight_subscriber = self.create_subscription(
            Bool, 
            "/start_flight",
            self.start_flight,
            10
        )
        self.start_scan_publisher = self.create_publisher(
            Bool, 
            "/fcu/out/start_scan", 
            10
        )
        self.set_external_engage_callback(self.publish_scan_start)
        self.init_heartbeat()

    def publish_scan_start(self):
        """
        Publishes a message to start the scan.
        """
        self.get_logger().info("Publishing start scan command.")
        msg = Bool()
        msg.data = True
        self.start_scan_publisher.publish(msg)
        self.get_logger().info("Published start scan command.")

    def helix_params_callback(self, data):
        self.get_logger().info(f"Received helix parameters: {data.data}")
        self.path = Helix(rate=self.rate)
        self.flight_params = {
            "helix_height": data.data[1],
            "helix_radius": data.data[0],
            "helix_num_passes": data.data[2],
            "helix_start_height": data.data[3],
            "helix_rate": 15.0 # Default rate, can be adjusted
        }
        self.path.set_params(
            helix_height=self.flight_params["helix_height"],
            helix_radius=self.flight_params["helix_radius"],
            helix_num_passes=self.flight_params["helix_num_passes"],
            helix_start_height=self.flight_params["helix_start_height"],
            helix_rate=self.flight_params["helix_rate"]
        )

        self.update_path(self.path)


def main(args=None) -> None:
    """
    Main function to initialize and run the HelixNode
    """
    rclpy.init(args=args)
    helix_node = HelixNode()
    try:
        rclpy.spin(helix_node)
    except KeyboardInterrupt:
        helix_node.get_logger().info("Keyboard interrupt received, shutting down...")
    finally:
        helix_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"An error occurred in main: {e}")

    