import os
import rclpy
from rclpy.node import Node
from threading import Thread
from flask import Flask, render_template
import std_msgs.msg

# Add Flask-SocketIO
from flask_socketio import SocketIO

def create_flask_app(node):
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

    app = Flask(__name__, template_folder=template_dir, static_folder=static_dir)
    socketio = SocketIO(app)

    @app.route('/')
    def home():
        return render_template('index.html')

    @app.route('/widgets/helix')
    def helix_widget():
        return render_template('widgets/helix.html')

    @socketio.on('radius_changed')
    def handle_radius_changed(data):
        try:
            radius = float(data)
            # node.publish_radius(radius)
            print(f"radius_changed event received: {radius}")
            print(type(radius))
            node.publish_radius(radius)  # Publish the radius to ROS topic
        except (ValueError, TypeError):
            print(f"Invalid radius value received: {data}")

    return app, socketio

class FlaskServerNode(Node):
    def __init__(self):
        super().__init__('flask_server_node')
        self.get_logger().info('FlaskServerNode initialized.')
        self.radius_publisher = self.create_publisher(
            std_msgs.msg.Float32, 'radius', 10)

    def publish_radius(self, radius):
        msg = std_msgs.msg.Float32()
        msg.data = radius
        self.get_logger().info(f'Publishing radius: {radius}')
        self.radius_publisher.publish(msg)

def run_flask(socketio, app):
    socketio.run(app, host='0.0.0.0', port=5000)

def main(args=None):
    rclpy.init(args=args)
    node = FlaskServerNode()
    app, socketio = create_flask_app(node)
    flask_thread = Thread(target=run_flask, args=(socketio, app))
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
