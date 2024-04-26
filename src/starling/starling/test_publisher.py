from rclpy.node import Node
import rclpy
from std_msgs.msg import Bool, String, Float32
from rclpy.qos import qos_profile_system_default
from voxl_reset_qvio import VOXLQVIOController


class TestPublisher(Node):
    def __init__(self) -> None:
        super().__init__("test_publisher_node")

        self.ready_pub = self.create_publisher(
            Bool, 
            "/host/gui/out/ready", 
            qos_profile_system_default
        )
        self.radius_pub = self.create_publisher(
            Float32, 
            "/host/gui/out/radius", 
            qos_profile_system_default
        )
        self.object_height_pub = self.create_publisher(
            Float32, 
            "/host/gui/out/object_height",
            qos_profile_system_default
        )
        self.start_height_pub = self.create_publisher(
            Float32, 
            "/host/gui/out/start_height", 
            qos_profile_system_default
        )
        self.scan_title_pub = self.create_publisher(
            String, 
            "/host/gui", 
            qos_profile_system_default
        )

        self.qvio_reset = VOXLQVIOController()
    
    def run(self):
        self.qvio_reset.reset()
        self.ready_pub.publish(self.create_bool(False))
        self.radius_pub.publish(self.create_float32(0.75))
        self.object_height_pub.publish(self.create_float32(0.6))
        self.start_height_pub.publish(self.create_float32(0.5))
        self.scan_title_pub.publish(self.create_string("Test Data"))
        self.ready_pub.publish(self.create_bool(True))


    def create_float32(self, val):
        res = Float32()
        res.data = val
        return res
    
    def create_bool(self, val):
        res = Bool()
        res.data = val
        return res

    def create_string(self, val):
        res = String()
        res.data = val
        return res

def main(args=None) -> None:
    rclpy.init(args=args)
    tp = TestPublisher()
    _ = input("Enter when ready.")
    tp.run()

if __name__ == "__main__":
    main()
    
        

