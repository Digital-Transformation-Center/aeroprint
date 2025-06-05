import os
import rclpy
from rclpy.node import Node
from threading import Thread
from flask import Flask, render_template
import std_msgs.msg
from std_msgs.msg import String
import json

from flask_socketio import SocketIO

class FlaskWebApp:
    def __init__(self, node):
        # Setup template/static dirs
        package_dir = os.path.dirname(os.path.realpath(__file__))
        template_dir = os.path.join(package_dir, 'templates')
        static_dir = os.path.join(package_dir, 'static')
        try:
            from ament_index_python.packages import get_package_share_directory
            ros_package_dir = get_package_share_directory('host')
            template_dir = os.path.join(ros_package_dir, 'web', 'templates')
            static_dir = os.path.join(ros_package_dir, 'web', 'static')
            
        except ImportError:
            ros_package_dir = package_dir

        self.app = Flask(__name__, template_folder=template_dir, static_folder=static_dir)
        self.socketio = SocketIO(self.app)
        self.node = node

        # Allow node to call emit_heartbeat
        self.node.flask_web_app = self

        self.node.get_logger().info(f"Using template directory: {template_dir}")
        self.node.get_logger().info(f"Using static directory: {static_dir}")

        self.setup_routes()
        self.setup_socketio_handlers()

    def emit_heartbeat(self, ok):
        self.socketio.emit('heartbeat', {'ok': ok})

    def setup_routes(self):
        @self.app.route('/')
        def home():
            return render_template('index.html')

        @self.app.route('/widgets/helix')
        def helix_widget():
            return render_template('widgets/helix.html')

        @self.app.route('/widgets/flight_config')
        def flight_config_widget():
            return render_template('widgets/flight_config.html')

    def setup_socketio_handlers(self):
        @self.socketio.on('shutdown_test_node')
        def handle_shutdown_test_node():
            self.node.shutdown_test_node()
            self.emit_notification('Shutdown signal sent to test_node.', warning='bad')

        @self.socketio.on('save_flight_config')
        def handle_save_flight_config(data):
            self.node.publish_params(data)
            print(f"save_flight_config event received with data: {data}")
            self.node.get_logger().info(f'Saving flight config data: {data}')
            self.emit_status('flight_config_saved')

    def status_callback(self, data):
        self.emit_notification('ROS Callback functions.')
        if data == 3:
            self.emit_notification("Loaded Flight Parameters.")
        elif data == 4:
            self.emit_notification("Failed to Load Flight Parameters", warning='bad')

    def emit_status(self, status):
        self.socketio.emit('server_status', {'status': status})

    def emit_notification(self, message, warning='good'):
        self.socketio.emit('notification', {'message': message, 'warning': warning})

    def run(self):
        self.socketio.run(self.app, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True)

class FlaskServerNode(Node):
    def __init__(self):
        super().__init__('flask_server_node')
        self.get_logger().info('FlaskServerNode initialized.')
        self.radius_publisher = self.create_publisher(
            std_msgs.msg.Float32, 'radius', 10)
        
        self.drone_status_subscriber = self.create_subscription(
            std_msgs.msg.Int8, 'drone_status', self.drone_status_callback, 10
        )
        self.param_publisher = self.create_publisher(
            std_msgs.msg.String, 'helix_params', 10 
        )
        self.flask_status_callback = None

        # Heartbeat subscriber
        self.heartbeat_ok = False
        self.create_subscription(
            std_msgs.msg.Int8, 'heartbeat', self.heartbeat_callback, 10
        )
        import threading
        self._heartbeat_timeout = 0.5  # seconds
        self._heartbeat_lock = threading.Lock()
        self._heartbeat_last = 0
        self._heartbeat_running = True
        self._heartbeat_thread = threading.Thread(target=self._heartbeat_monitor, daemon=True)
        self._heartbeat_thread.start()

    def shutdown_test_node(self):
        # Publish a shutdown message to a topic
        if not hasattr(self, 'shutdown_publisher'):
            self.shutdown_publisher = self.create_publisher(std_msgs.msg.Int8, 'test_node_shutdown', 10)
        msg = std_msgs.msg.Int8()
        msg.data = 99  # Arbitrary shutdown codes
        self.get_logger().info('Publishing shutdown signal to test_node.')
        self.shutdown_publisher.publish(msg)

    def heartbeat_callback(self, msg):
        with self._heartbeat_lock:
            self.heartbeat_ok = True
            import time
            self._heartbeat_last = time.time()
        # Emit to web clients
        if hasattr(self, 'flask_web_app'):
            self.flask_web_app.emit_heartbeat(True)

    def _heartbeat_monitor(self):
        import time
        while self._heartbeat_running:
            now = time.time()
            with self._heartbeat_lock:
                ok = (now - self._heartbeat_last) < self._heartbeat_timeout
                self.heartbeat_ok = ok
            if hasattr(self, 'flask_web_app'):
                self.flask_web_app.emit_heartbeat(ok)
            time.sleep(0.1)  # Check every 100ms

    def publish_radius(self, radius):
        msg = std_msgs.msg.Float32()
        msg.data = radius
        self.get_logger().info(f'Publishing radius: {radius}')
        self.radius_publisher.publish(msg)

    def publish_params(self, data):
        msg = String()
        msg.data = json.dumps(data)
        self.param_publisher.publish(msg)


    def drone_status_callback(self, msg):
        if self.flask_status_callback:
            self.flask_status_callback(msg.data)
            self.get_logger().info("Calling flask callback.")
        else:
            self.get_logger().info("Status callback not attached.")

    def attach_status_callback(self, fn):
        self.flask_status_callback = fn

def run_flask(flask_app):
    flask_app.run()

def main(args=None):
    rclpy.init(args=args)
    node = FlaskServerNode()
    flask_app = FlaskWebApp(node)
    node.attach_status_callback(flask_app.status_callback)
    flask_thread = Thread(target=run_flask, args=(flask_app,))
    flask_thread.daemon = True
    flask_thread.start()
    node.get_logger().info('Flask server started in a separate thread.')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        node.get_logger().info('ROS node shutdown.')
        # Flask thread is daemon, will exit with main process

if __name__ == '__main__':
    main()
