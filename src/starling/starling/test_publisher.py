from rclpy.node import Node
import rclpy
from std_msgs.msg import Bool, String, Float32
from rclpy.qos import qos_profile_system_default
# from voxl_reset_qvio import VOXLQVIOController


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
            "/host/gui/out/scan_title", 
            qos_profile_system_default
        )
        self.scan_start_pub = self.create_publisher(
            Bool, 
            "/starling/out/fc/scan_start", 
            qos_profile_system_default
        )
        self.scan_end_pub = self.create_publisher(
            Bool, 
            "/starling/out/fc/scan_end", 
            qos_profile_system_default
        )

        self.ready_pub.publish(self.create_bool(False))
        self.radius_pub.publish(self.create_float32(0.0))
        self.object_height_pub.publish(self.create_float32(0.0))
        self.start_height_pub.publish(self.create_float32(0.0))
        self.scan_title_pub.publish(self.create_string(""))
        
        # self.qvio_reset = VOXLQVIOController()
    
    def run(self):
        self.reset()
        self.radius_pub.publish(self.create_float32(0.75))
        self.object_height_pub.publish(self.create_float32(0.6))
        self.start_height_pub.publish(self.create_float32(0.5))
        self.scan_title_pub.publish(self.create_string("Test Data"))
        self.ready_pub.publish(self.create_bool(True))

    def start_flight(self):
        # self.qvio_reset.reset()
        None

    def start_scan(self):
        self.scan_start_pub.publish(self.create_bool(True))

    def end_scan(self):
        self.scan_end_pub.publish(self.create_bool(True))

    def reset(self):
        self.ready_pub.publish(self.create_bool(False))
        # self.qvio_reset.reset()

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
    while True:
        res = input("Enter Command:")
        if res == "ready":
            tp.run()
        elif res == "start-flight":
            tp.start_flight()
        elif res == "start-scan":
            tp.start_scan()
        elif res == "end-scan":
            tp.end_scan()
        elif res == "reset":
            tp.reset()

    _ = input("Enter when ready.")
    tp.run()
    _ = input("Enter to start flight.")
    tp.start_flight()
    _ = input("Enter to begin scan.")
    tp.start_scan()
    _ = input("Enter to end scan.")
    tp.end_scan()


if __name__ == "__main__":
    main()
    
        

