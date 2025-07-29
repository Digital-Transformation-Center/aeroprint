import os
import rclpy
from rclpy.node import Node
from threading import Thread
from flask import Flask, render_template, send_from_directory, jsonify
import std_msgs.msg
from std_msgs.msg import String, Float32MultiArray
import json
from rclpy.qos import qos_profile_system_default

from flask_socketio import SocketIO

from starling.flightdefs.flight_status_codes import *
from host.web.file_manager import FileManager

ASSETS_DIR = '/var/lib/aeroprint/'

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
            selected_size = "MED"
            command = "STOP"
            stage = "Preflight"
            # starling_status = is_starling_reachable()

            # Change this to check the heartbeat status
            starling_status = True  # Assume Starling is reachable for now

            # Load last command/size if available
            if os.path.exists("command.json") and os.path.getsize("command.json") > 0:
                try:
                    with open("command.json") as f:
                        data = json.load(f)
                        selected_size = data.get("size", selected_size)
                        command = data.get("command", command)
                except Exception:
                    pass

            # Load last stage if available
            if os.path.exists("status.json") and os.path.getsize("status.json") > 0:
                try:
                    with open("status.json") as f:
                        data = json.load(f)
                        stage = data.get("stage", stage)
                except Exception:
                    pass

            return render_template("index.html",
                selected_size=selected_size,
                command=command,
                stage=stage,
                starling=starling_status
            )
        # def home():
        #     # Pass units to the template
        #     return render_template('index.html', units=self.units)
        


        @self.app.route('/dev')
        def dev():
            return render_template('dev.html', units=self.units)

        # @self.app.route('/widgets/helix')
        # def helix_widget():
        #     return render_template('widgets/helix.html')

        @self.app.route('/widgets/flight_config')
        def flight_config_widget():
            return render_template('widgets/flight_config.html', units=self.units)
        
        @self.app.route('/assets/<path:filename>')
        def serve_assets(filename):
            # send_from_directory safely serves files from the specified directory
            # It handles security concerns like directory traversal
            try:
                return send_from_directory('/var/lib/aeroprint', filename)
            except FileNotFoundError:
                # Handle cases where the requested file doesn't exist
                return "File not found", 404
            
        @self.app.route('/api/list_assets/<path:subpath>')
        @self.app.route('/api/list_assets/') # For listing the root of assets/
        def list_assets(subpath=''):
            target_dir = os.path.join(ASSETS_DIR, subpath)

            # Basic security check: ensure the target_dir is still within ASSETS_DIR
            # This prevents directory traversal attacks if someone tries ../../
            if not os.path.abspath(target_dir).startswith(os.path.abspath(ASSETS_DIR)):
                return jsonify({"error": "Access denied"}), 403

            if not os.path.isdir(target_dir):
                return jsonify({"error": "Directory not found"}), 404

            files_and_dirs = []
            for entry in os.listdir(target_dir):
                entry_path = os.path.join(target_dir, entry)
                relative_path = os.path.join(subpath, entry) # Path relative to ASSETS_DIR

                if os.path.isfile(entry_path):
                    files_and_dirs.append({
                        "name": entry,
                        "path": f"/assets/{relative_path.replace(os.sep, '/')}", # Frontend URL
                        "type": "file"
                    })
                elif os.path.isdir(entry_path):
                    files_and_dirs.append({
                        "name": entry,
                        "path": f"/api/list_assets/{relative_path.replace(os.sep, '/')}", # API URL for subdirectory
                        "type": "directory"
                    })
            return jsonify(files_and_dirs)
        
        @self.app.route('/api/get_current_scan_num')
        def get_current_scan_num():
            """API endpoint to get the current scan number."""
            scan_num = self.node.fm.get_id()
            return jsonify({"scan_num": scan_num})
        

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

        @self.socketio.on('select_size')
        def handle_select_size(data):
            self.node.get_logger().info(f"Size selected: {data.get('size', 'MED')}")
            size = data.get('size', 'MED')
            flight_config = {}
            if size == 'SM':
                flight_config = {
                    'radius': 0.2,
                    'height': 0.3,
                    'turns': 2,
                    'startHeight': 0.47
                }
            elif size == 'MED':
                flight_config = {
                    'radius': 0.3,
                    'height': 0.5,
                    'turns': 3,
                    'startHeight': 0.47
                }
            elif size == 'LG':
                flight_config = {
                    'radius': 0.4,
                    'height': 0.6,
                    'turns': 4,
                    'startHeight': 0.47
                }
            self.node.publish_params(flight_config)
            self.socketio.emit('path_values_changed', {'radius': flight_config['radius'],
                                                       'height': flight_config['height'],
                                                       'turns': flight_config['turns'],
                                                       'startHeight': flight_config['startHeight']})

    def status_callback(self, data):
        self.emit_notification('ROS Callback functions.')
        if data == 3:
            self.emit_notification("Loaded Flight Parameters.")
        elif data == 4:
            self.emit_notification("Failed to Load Flight Parameters", warning='bad')

        if data == 1:
            self.emit_status('DISARMED')
        elif data == 2:
            self.emit_status('ENGAGED')
        elif data == 3:
            self.emit_status('ARMED')
        elif data == 4:
            self.emit_status('LANDING')
        elif data == 5:
            self.emit_status('PATH_LOADED') 

    def emit_ui_phase(self, phase):
        self.socketio.emit('update_phase', {'phase': phase})
    
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
        self.scan_end_subscriber = self.create_subscription(
            std_msgs.msg.Bool, '/fcu/out/scan_end', self.scan_end_callback, 10
        )
        self.pcd_end_subscriber = self.create_subscription(
            std_msgs.msg.Bool, '/host/out/pcpp/export_complete', self.pc_complete_callback, 10
        )
        self.mesh_complete_subscriber = self.create_subscription(
            std_msgs.msg.Bool, '/host/out/mesher/mesh_complete', self.mesh_complete_callback, 10
        )
        self.slicing_complete_subscriber = self.create_subscription(
            std_msgs.msg.Bool, '/host/out/slicer/slicing_complete', self.slicing_complete_callback, 10
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
                self.flask_web_app.emit_ui_phase('Scanning')
        elif status == FLIGHT_ARMED:
            if hasattr(self, 'flask_web_app'):
                self.flask_web_app.emit_status('flight_armed')
        elif status == FLIGHT_LANDING:
            if hasattr(self, 'flask_web_app'):
                self.flask_web_app.emit_status('flight_landing')
        elif status == FLIGHT_ERROR:
            if hasattr(self, 'flask_web_app'):
                self.flask_web_app.emit_status('flight_error')

    def slicing_complete_callback(self, msg):
        if hasattr(self, 'flask_web_app'):
            self.flask_web_app.emit_ui_phase('Printing')
    
    def mesh_complete_callback(self, msg):
        if hasattr(self, 'flask_web_app'):
            self.flask_web_app.emit_ui_phase('Slicing')
    
    def pc_complete_callback(self, msg):
        if hasattr(self, 'flask_web_app'):
            self.flask_web_app.emit_ui_phase('Meshing')
    
    def scan_end_callback(self, msg):
        if hasattr(self, 'flask_web_app'):
            self.flask_web_app.emit_ui_phase('Processing')

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
