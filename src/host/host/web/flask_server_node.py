import os
import rclpy
from rclpy.node import Node
from threading import Thread
from flask import Flask, render_template
import std_msgs.msg
from std_msgs.msg import String, Float32MultiArray
import json
from rclpy.qos import qos_profile_system_default

from flask_socketio import SocketIO

from starling.flightdefs.flight_status_codes import *
from host.web.file_manager import FileManager

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
        self.units = 'meters'

    def emit_heartbeat(self, ok):
        self.socketio.emit('heartbeat', {'ok': ok})

    def setup_routes(self):
        @self.app.route('/')
        def home():
            # Pass units to the template
            return render_template('index.html', units=self.units)

        # @self.app.route('/widgets/helix')
        # def helix_widget():
        #     return render_template('widgets/helix.html')

        @self.app.route('/widgets/flight_config')
        def flight_config_widget():
            return render_template('widgets/flight_config.html', units=self.units)
        

    def setup_socketio_handlers(self):
        @self.socketio.on('disconnect')
        def handle_disconnect():
            self.node.get_logger().info('Client disconnected.')
            self.node.request_land_flight()  # Request landing on disconnect
            
        @self.socketio.on('shutdown_test_node')
        def handle_shutdown_test_node():
            self.node.shutdown_test_node()
            self.emit_notification('Shutdown signal sent to Test Node.', warning='bad')

        @self.socketio.on('save_flight_config')
        def handle_save_flight_config(data):
            self.node.publish_params(data)
            print(f"save_flight_config event received with data: {data}")
            self.node.get_logger().info(f'Saving flight config data: {data}')
            self.emit_status('flight_config_saved')

        @self.socketio.on('start_flight')
        def handle_start_flight():
            self.node.start_flight()
            self.emit_notification('Stand clear. Flight starting.', warning='good')

        @self.socketio.on('land_flight')
        def handle_land_flight():
            self.node.get_logger().info('Landing flight.')
            self.node.request_land_flight()

        @self.socketio.on('set_units')
        def handle_set_units(data):
            self.units = data.get('units', 'meters')
            self.emit_notification(f"Units set to {self.units}.", warning='warn')
            self.socketio.emit('units_changed', {'units': self.units})

        @self.socketio.on('notification')
        def handle_notification(data):
            message = data.get('message', 'No message provided')
            warning = data.get('warning', 'warn')
            self.emit_notification(message, warning=warning)
            self.node.get_logger().info(f"Notification: {message} (Warning: {warning})")

        @self.socketio.on('path_values_changed')
        def handle_path_values_changed(data):
            self.emit_status('params_changed')

    def status_callback(self, data):
        self.emit_notification('ROS Callback functions.')
        if data == 3:
            self.emit_notification("Loaded Flight Parameters.")
        elif data == 4:
            self.emit_notification("Failed to Load Flight Parameters", warning='bad')

    def emit_status(self, status):
        self.socketio.emit('flight_status', {'status': status})

    def emit_notification(self, message, warning='good'):
        self.socketio.emit('notification', {'message': message, 'warning': warning})

    def run(self):
        self.socketio.run(self.app, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True)

class FlaskServerNode(Node):
    def __init__(self):
        super().__init__('flask_server_node')
        self.fm = FileManager()
        self.get_logger().info('FlaskServerNode initialized.')
        self.radius_publisher = self.create_publisher(
            std_msgs.msg.Float32, 'radius', 10
        )

        self.param_publisher = self.create_publisher(
            Float32MultiArray, 'helix_params', 10 
        )

        self.start_flight_publisher = self.create_publisher(
            std_msgs.msg.Bool, 'start_flight', 10
        )

        self.status_subscriber = self.create_subscription(
            std_msgs.msg.Int8, '/fcu/out/status', self.flight_status_callback, 10
        )
        self.scan_id_publisher = self.create_publisher(
            std_msgs.msg.Int32, '/web/scan_id', qos_profile_system_default
        )
        self.pcd_directory_publisher = self.create_publisher(
            String, '/web/pcd_directory', qos_profile_system_default
        )
        self.log_directory_publisher = self.create_publisher(
            String, '/web/log_directory', qos_profile_system_default
        )

        self.flask_status_callback = None

        # Heartbeat subscriber
        self.heartbeat_ok = False
        self.create_subscription(
            std_msgs.msg.Int8, '/fcu/out/heartbeat', self.heartbeat_callback, 10
        )
        self.heartbeat_publisher = self.create_publisher(
            std_msgs.msg.Int8, '/fcu/in/heartbeat', 10
        )
        self.est_time_subscriber = self.create_subscription(
            std_msgs.msg.Float32, '/fcu/out/est_time', self.est_time_callback, 10
        )
        self.time_remaining_subscriber = self.create_subscription(
            std_msgs.msg.Float32, '/fcu/out/time_remaining', self.time_remaining_callback, 10
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
        self.heartbeat_publisher.publish(msg)
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
        self.get_logger().info("Web Node received parameters.")
        radius = data['radius']
        height = data['height']
        turns = data['turns']
        start_height = data['startHeight']
        self.get_logger().info(f"Received radius: {radius}")

        print(f"Radius: {data['radius']}")
        # radius: flight_path_radius,
        # height: flight_path_height,
        # turns: flight_path_turns,
        # startHeight: flight_path_start_height,
        msg = Float32MultiArray()
        msg.data = [float(radius), float(height), float(turns), float(start_height)]

        self.param_publisher.publish(msg)


    def flight_status_callback(self, msg):
        status = msg.data
        if status == FLIGHT_DISARMED:
            if hasattr(self, 'flask_web_app'):
                self.flask_web_app.emit_status('idle')
        elif status == FLIGHT_PATH_LOADED:
            if hasattr(self, 'flask_web_app'):
                self.flask_web_app.emit_status('param_receive')
        elif status == FLIGHT_ENGAGED:
            self.get_logger().info("Flight engaged.")
            if hasattr(self, 'flask_web_app'):
                self.flask_web_app.emit_status('flight_engaged')
        elif status == FLIGHT_ARMED:
            if hasattr(self, 'flask_web_app'):
                self.flask_web_app.emit_status('flight_armed')
        elif status == FLIGHT_LANDING:
            if hasattr(self, 'flask_web_app'):
                self.flask_web_app.emit_status('flight_landing')
        elif status == FLIGHT_ERROR:
            if hasattr(self, 'flask_web_app'):
                self.flask_web_app.emit_status('flight_error')

    def est_time_callback(self, msg):
        if hasattr(self, 'flask_web_app'):
            self.flask_web_app.socketio.emit('flight_time_estimate', {'time': msg.data})

    def time_remaining_callback(self, msg):
        if hasattr(self, 'flask_web_app'):
            self.flask_web_app.socketio.emit('flight_time_remaining', {'time': msg.data})

    def attach_status_callback(self, fn):
        self.flask_status_callback = fn

    def start_flight(self):
        self.fm.create_new_folder()
        self.pcd_directory_publisher.publish(
            String(data=self.fm.get_pcd_folder(self.fm.get_id()))
        )
        self.log_directory_publisher.publish(
            String(data=self.fm.get_logs_folder(self.fm.get_id()))
        )
        msg = std_msgs.msg.Bool()
        msg.data = True
        self.start_flight_publisher.publish(msg)
        self.get_logger().info('Flight started.')

    def request_land_flight(self):
        msg = std_msgs.msg.Bool()
        msg.data = False
        self.start_flight_publisher.publish(msg)
        self.get_logger().info('Landing flight requested.')


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
