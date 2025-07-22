__author__ = "Ryan Kuederle"
__email__ = "ryan@rkuederle.com"
__version__ = "0.1.0"

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool, Int8, Float32
# from starling.flightdefs.flight_status_codes import STATUS_TITLES, STATUS_DESCRIPTIONS
import threading
import time

class GUI_FCU_Plugin(Node):
    def __init__(self):
        super().__init__('gui_fcu_plugin')

        self.param_publisher = self.create_publisher(
            Float32MultiArray, 'helix_params', 10 
        )

        self.start_flight_publisher = self.create_publisher(
            Bool, 'start_flight', 10
        )

        self.status_subscriber = self.create_subscription(
            Int8, '/fcu/out/status', self._flight_status_callback, 10
        )

        self.flask_status_callback = None

        # Heartbeat subscriber
        self.heartbeat_ok = False
        self.create_subscription(
            Int8, '/fcu/out/heartbeat', self._heartbeat_callback, 10
        )
        self.heartbeat_publisher = self.create_publisher(
            Int8, '/fcu/in/heartbeat', 10
        )
        self.est_time_subscriber = self.create_subscription(
            Float32, '/fcu/out/est_time', self._est_time_callback, 10
        )
        self.time_remaining_subscriber = self.create_subscription(
            Float32, '/fcu/out/time_remaining', self._time_remaining_callback, 10
        )

        # User callbacks
        self._user_est_time_callback = None
        self._user_time_remaining_callback = None
        self._user_heartbeat_alive_callback = None
        self._user_heartbeat_dead_callback = None
        self._user_status_code_callback = None
        self._user_status_description_callback = None
        self._user_status_code_callback = None
        self._user_status_title_callback = None
        self._user_flight_path_loaded_callback = None
        self._user_flight_path_not_loaded_callback = None

        # Heartbeat data
        self._heartbeat_timeout = 0.5
        self._heartbeat_lock = threading.Lock()
        self._heartbeat_last = 0
        self._thread_running = True
        self._flight_path_loaded = False
        self._main_thread = threading.Thread(target=self._main_loop, daemon=True)

    def run(self):
        self._main_thread.start()

    def kill(self):
        """Stop the unified main thread."""
        self._thread_running = False
        rclpy.shutdown()

    def set_helix_params(self, radius, height, turns, start_height):
        msg = Float32MultiArray()
        msg.data = [float(radius), float(height), float(turns), float(start_height)]
        self._flight_path_loaded = False
        self.param_publisher.publish(msg)

    def start_flight(self):
        """Start the flight by publishing a message to the start_flight topic."""
        msg = Bool()
        msg.data = True
        self.start_flight_publisher.publish(msg)

    def land(self):
        """Land the drone by publishing a message to the start_flight topic."""
        msg = Bool()
        msg.data = False
        self.start_flight_publisher.publish(msg)

    def flight_path_loaded(self):
        return self._flight_path_loaded

    """
    ROS callbacks for FCU topics
    """

    def _flight_status_callback(self, msg):
        if not self._user_status_code_callback == None:
            self._user_status_code_callback(msg.data)
        if not self._user_status_description_callback == None:
            self._user_status_description_callback(self._status_get_description(msg.data))
        if not self._user_status_title_callback == None:
            self._user_status_title_callback(self._status_get_title(msg.data))
        self._status_action(msg.data)

    def _heartbeat_callback(self, msg):
        self.heartbeat_publisher.publish(msg)
        self._process_heartbeat(msg.data)

    def _est_time_callback(self, msg):
        if not self._user_est_time_callback == None:
            self._user_est_time_callback(msg.data)

    def _time_remaining_callback(self, msg):
        if not self._user_time_remaining_callback == None:
            self._user_time_remaining_callback(msg.data)

    """
    Methods for user to define callbacks
    """

    # Heartbeat management
    def set_heartbeat_alive_callback(self, callback, obj):
        """Callback params none"""
        self._user_heartbeat_alive_callback = getattr(obj, callback)


    def set_heartbeat_dead_callback(self, callback, obj):
        """Callback params none"""
        self._user_heartbeat_dead_callback = getattr(obj, callback)

    # FCU time estimation callback
    def set_est_time_callback(self, callback, obj):
        """Callback params Float32: time remaining (s)"""
        self._user_est_time_callback = getattr(obj, callback)

    def set_time_remaining_callback(self, callback, obj):
        """Callback params Float32: time remaining (s)"""
        self._user_time_remaining_callback = getattr(obj, callback)

    # FCU status callbacks
    def set_status_description_callback(self, callback, obj):
        """Callback params String: description"""
        self._user_status_description_callback = getattr(obj, callback)

    def set_status_title_callback(self, callback, obj):
        """Callback params String: title"""
        self._user_status_title_callback = getattr(obj, callback)

    def set_status_code_callback(self, callback, obj):
        """Callback params Int8: code"""
        self._user_status_code_callback = getattr(obj, callback)

    def set_flight_path_loaded_callback(self, callback, obj):
        """Callback params none"""
        self._user_flight_path_loaded_callback = getattr(obj, callback)

    def set_flight_path_not_loaded_callback(self, callback, obj):
        """Callback params none"""
        self._user_flight_path_not_loaded_callback = getattr(obj, callback)

    """
    Helper functions
    """

    # Status management
    def _status_action(self, status):
        match status:
            case 5:
                self._flight_path_loaded = True
                if not self._user_flight_path_loaded_callback == None:
                    self._user_flight_path_loaded_callback()

            case 6:
                self._flight_path_loaded = False
                if not self._user_flight_path_not_loaded_callback == None:
                    self._user_flight_path_not_loaded_callback()


    def _status_get_title(self, status):
        # return STATUS_TITLES[status]
        return None

    def _status_get_description(self, status):
        # return STATUS_DESCRIPTIONS[status]
        return None
    
    def _process_heartbeat(self, data):
        with self._heartbeat_lock:
            self.heartbeat_ok = True
            import time
            self._heartbeat_last = time.time()
    
    def _main_loop(self):
        """
        Unified thread for ROS spinning and heartbeat monitoring.
        """
        while self._thread_running:
            # Spin ROS node
            try:
                rclpy.spin_once(self, timeout_sec=0.01)
            except Exception as e:
                self.get_logger().error(f"Error in ROS spin: {e}")
            # Heartbeat check
            now = time.time()
            with self._heartbeat_lock:
                ok = (now - self._heartbeat_last) < self._heartbeat_timeout
                self.heartbeat_ok = ok
            if ok and not self._user_heartbeat_alive_callback == None:
                self._user_heartbeat_alive_callback()
            elif not ok and not self._user_heartbeat_dead_callback == None:
                self._user_heartbeat_dead_callback()
            time.sleep(0.05)  # 20Hz
