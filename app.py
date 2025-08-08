from flask import Flask, render_template, send_file, jsonify
from flask_socketio import SocketIO, emit
import os, json, subprocess, platform
import threading 
import time
import struct
import random


app = Flask(__name__)
socketio = SocketIO(app)

# Size → Flight parameter mapping
SIZE_TO_RADIUS = {
    "SM": 1.0,
    "MED": 2.0,
    "LG": 3.5
}

def is_starling_reachable():
    try:
        param = "-n" if platform.system().lower() == "windows" else "-c"
        result = subprocess.run(
            ["ping", param, "1", "m0054"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        return result.returncode == 0
    except Exception:
        return False

@app.route("/")
def home():
    selected_size = "MED"
    command = "STOP"
    stage = "Preflight"
    starling_status = is_starling_reachable()

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

# Missing API endpoints that your frontend needs
@app.route("/api/get_current_scan_num")
def get_current_scan_num():
    """
    API endpoint to get the current scan number
    """
    scan_num = get_latest_scan_number()
    return {"scan_num": scan_num}

@app.route("/api/list_assets/<int:scan_num>/<asset_type>")
def list_assets(scan_num, asset_type):
    """
    List assets for a given scan number and asset type
    """
    if asset_type == "pcd":
        # Try to find real files first
        base_paths = [
            f"/var/lib/aeroprint/scans/{scan_num}/pcd",
            f"./scan_{scan_num}/pcd",
            f"./scans/{scan_num}/pcd"
        ]
        
        for base_path in base_paths:
            if os.path.exists(base_path):
                try:
                    files = []
                    for filename in os.listdir(base_path):
                        if filename.endswith('.pcd'):
                            files.append({
                                "name": filename,
                                "type": "file",
                                "path": f"/api/serve_file/{scan_num}/pcd/{filename}"
                            })
                    if files:
                        return files
                except Exception as e:
                    print(f"Error listing files in {base_path}: {e}")
        
        # If no real files found, return mock data for testing
        mock_assets = [
            {
                "name": "combined_filtered.pcd",
                "type": "file",
                "path": f"/api/serve_file/{scan_num}/pcd/combined_filtered.pcd"
            }
        ]
        return mock_assets
    
    return []

@app.route("/api/serve_file/<int:scan_num>/<asset_type>/<filename>")
def serve_file(scan_num, asset_type, filename):
    """
    Serve actual PCD files or mock data
    """
    # Try to serve real file first
    file_paths = [
        f"/var/lib/aeroprint/scans/{scan_num}/{asset_type}/{filename}",
        f"./scan_{scan_num}/{asset_type}/{filename}",
        f"./scans/{scan_num}/{asset_type}/{filename}"
    ]
    
    for file_path in file_paths:
        if os.path.exists(file_path):
            return send_file(file_path)
    
    # If no real file found, generate mock data for testing
    print(f"Real file not found, generating mock data for {filename}")
    mock_pcd_data = generate_mock_pcd_data(1000)  # 1000 points
    return mock_pcd_data, 200, {'Content-Type': 'application/octet-stream'}

def generate_mock_pcd_data(num_points):
    """
    Generate mock PCD file data for testing
    """
    # PCD header
    header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {num_points}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {num_points}
DATA binary
"""
    
    # Generate random points in a cube shape
    points = []
    for i in range(num_points):
        # Create cube-like distribution
        x = random.uniform(-1, 1)
        y = random.uniform(-1, 1) 
        z = random.uniform(-1, 1)
        
        # Pack as binary floats
        points.extend(struct.pack('fff', x, y, z))
    
    # Combine header and binary data
    header_bytes = header.encode('utf-8')
    binary_data = bytes(points)
    
    return header_bytes + binary_data

def update_phase(new_phase):
    # Update status.json
    with open("status.json", "w") as f:
        json.dump({"stage": new_phase}, f)
    # Emit to all connected clients
    socketio.emit('status_update', {'phase': new_phase})

def send_ready():
    subprocess.run([
        "ros2", "topic", "pub", "-1", "/host/gui/out/ready", "std_msgs/msg/Bool", '{"data": true}'
    ])

def send_radius(radius):
    subprocess.run([
        "ros2", "topic", "pub", "-1", "/host/gui/out/radius", "std_msgs/msg/Float32", f'{{"data": {radius}}}'
    ])

def emit_progress(percent):
    socketio.emit('progress_update', {'percent': percent})

# Example function to emit point cloud data
def emit_pointcloud(points_array):
    # points_array should be a flat list: [x1, y1, z1, x2, y2, z2, ...]
    socketio.emit('pointcloud_data', {'points': points_array})

def on_new_pointcloud(points):
    # points: a list of [x, y, z] lists or tuples
    # Flatten to [x1, y1, z1, x2, y2, z2, ...]
    flat_points = [coord for point in points for coord in point]
    emit_pointcloud(flat_points)

def save_pointcloud(points, filename="latest_pointcloud.ply"):
    # points: list of [x, y, z]
    with open(filename, "w") as f:
        f.write("ply\nformat ascii 1.0\nelement vertex {}\nproperty float x\nproperty float y\nproperty float z\nend_header\n".format(len(points)))
        for pt in points:
            f.write(f"{pt[0]} {pt[1]} {pt[2]}\n")

@app.route("/download_pointcloud")
def download_pointcloud():
    file_path = os.path.join(os.getcwd(), "latest_pointcloud.ply")
    return send_file(file_path, as_attachment=True)

@app.route("/download_mesh")
def download_mesh():
    file_path = os.path.join(os.getcwd(), "latest_mesh.ply")
    return send_file(file_path, as_attachment=True)

@app.route("/download_slicing")
def download_slicing():
    file_path = os.path.join(os.getcwd(), "latest_slices.gcode")
    return send_file(file_path, as_attachment=True)

def simulate_pointcloud_stream():
    import random
    num_points = 500
    delay = 0.02  # 20ms between points
    for _ in range(num_points):
        # Generate a random point on the surface of a cube (like your dev cube)
        face = random.randint(0, 5)
        x = (random.random() - 0.5) * 2
        y = (random.random() - 0.5) * 2
        z = (random.random() - 0.5) * 2
        if face == 0: x = 1
        if face == 1: x = -1
        if face == 2: y = 1
        if face == 3: y = -1
        if face == 4: z = 1
        if face == 5: z = -1
        # Emit as a single point (flat array)
        socketio.emit('pointcloud_point', {'point': [x, y, z]})
        time.sleep(delay)

@app.route("/simulate_pointcloud")
def simulate_pointcloud():
    threading.Thread(target=simulate_pointcloud_stream, daemon=True).start()
    return "Simulated point cloud streaming started!"

def ensure_scan_directories():
    """
    Ensure scan directories exist
    """
    base_dir = "/var/lib/aeroprint/scans"
    
    try:
        if not os.path.exists(base_dir):
            os.makedirs(base_dir, exist_ok=True)
            print(f"Created base scan directory: {base_dir}")
        
        # Create directory for current scan
        current_scan = get_latest_scan_number()
        scan_dir = f"{base_dir}/{current_scan}"
        pcd_dir = f"{scan_dir}/pcd"
        
        os.makedirs(pcd_dir, exist_ok=True)
        print(f"Ensured scan directories exist: {pcd_dir}")
        
    except Exception as e:
        print(f"Error creating scan directories: {e}")

def get_latest_scan_number():
    """
    Get the latest/current scan number from the file system or a counter file
    """
    try:
        # Option 1: Read from a scan counter file
        if os.path.exists("current_scan.json"):
            with open("current_scan.json", "r") as f:
                data = json.load(f)
                scan_num = data.get("current_scan", 1)
                return scan_num
        
        # Option 2: Find the highest numbered scan directory
        scans_dir = "/var/lib/aeroprint/scans"
        if os.path.exists(scans_dir):
            scan_dirs = [d for d in os.listdir(scans_dir) if d.isdigit()]
            if scan_dirs:
                return max(int(d) for d in scan_dirs)
        
        # Option 3: Check for scan directories in current directory
        current_dirs = [d for d in os.listdir(".") if d.startswith("scan_") and d.split("_")[1].isdigit()]
        if current_dirs:
            return max(int(d.split("_")[1]) for d in current_dirs)
            
        # Default to 1 if no scans found
        return 1
        
    except Exception as e:
        print(f"Error getting scan number: {e}")
        return 1

def increment_scan_number():
    """
    Increment the scan number (call this when starting a new scan)
    """
    current = get_latest_scan_number()
    new_scan = current + 1
    
    # Save the new scan number
    with open("current_scan.json", "w") as f:
        json.dump({"current_scan": new_scan}, f)
    
    # Create directories for the new scan
    try:
        scan_dir = f"/var/lib/aeroprint/scans/{new_scan}"
        pcd_dir = f"{scan_dir}/pcd"
        os.makedirs(pcd_dir, exist_ok=True)
        print(f"Created directories for scan {new_scan}: {pcd_dir}")
    except Exception as e:
        print(f"Error creating scan {new_scan} directories: {e}")
    
    return new_scan

# Add a function to listen for ROS scan completion
def listen_for_scan_completion():
    """
    Listen for scan completion from ROS and handle phase transitions
    """
    def ros_listener():
        while True:
            try:
                # Listen for scan end signal
                result = subprocess.run([
                    "ros2", "topic", "echo", "/scan/end", "--once"
                ], capture_output=True, text=True, timeout=5)
                
                if result.returncode == 0:
                    print("Scan end signal received from ROS")
                    update_phase("Landing")
                    
                    # Wait for post-processing to complete or timeout
                    post_processing_success = wait_for_post_processing()
                    
                    if post_processing_success:
                        print("Post-processing completed successfully")
                        update_phase("Meshing")
                        time.sleep(5)
                        update_phase("Slicing")
                        time.sleep(5)
                        update_phase("Printing")
                    else:
                        print("Post-processing failed or timed out, continuing anyway")
                        update_phase("Meshing")
                        time.sleep(3)
                        update_phase("Slicing")
                        time.sleep(3)
                        update_phase("Printing")
                
            except Exception as e:
                print(f"Error listening for scan completion: {e}")
            
            time.sleep(2)
    
    threading.Thread(target=ros_listener, daemon=True).start()

def wait_for_post_processing():
    """
    Wait for point cloud post-processing to complete
    """
    max_wait = 30  # 30 seconds max
    waited = 0
    
    while waited < max_wait:
        try:
            # Check if combined_filtered.pcd exists
            current_scan = get_latest_scan_number()
            combined_file = f"/var/lib/aeroprint/scans/{current_scan}/pcd/combined_filtered.pcd"
            
            if os.path.exists(combined_file) and os.path.getsize(combined_file) > 0:
                print(f"Post-processing complete! File: {combined_file}")
                return True
            
            # Check for any .pcd files as a fallback
            pcd_dir = f"/var/lib/aeroprint/scans/{current_scan}/pcd"
            if os.path.exists(pcd_dir):
                pcd_files = [f for f in os.listdir(pcd_dir) if f.endswith('.pcd')]
                if len(pcd_files) > 0:
                    print(f"Found {len(pcd_files)} PCD files, post-processing may be complete")
                    return True
                    
        except Exception as e:
            print(f"Error checking post-processing status: {e}")
            
        time.sleep(2)
        waited += 2
    
    print("Post-processing timeout reached")
    return False

# Add these missing function definitions that are called in main
def listen_to_drone_status():
    """
    Listen to drone status and update phases accordingly
    """
    def status_listener():
        current_phase = "Preflight"
        
        while True:
            try:
                # Check if drone is ready/armed
                result = subprocess.run([
                    "ros2", "topic", "echo", "/mavros/state", "--once"
                ], capture_output=True, text=True, timeout=2)
                
                if result.returncode == 0:
                    # Parse the status and determine phase
                    new_phase = determine_phase_from_status(result.stdout)
                    if new_phase != current_phase:
                        current_phase = new_phase
                        update_phase(new_phase)
                        print(f"Phase changed to: {new_phase}")
                
            except Exception as e:
                print(f"Error checking drone status: {e}")
            
            time.sleep(5)  # Check every 5 seconds
    
    # Start the listener in a background thread
    threading.Thread(target=status_listener, daemon=True).start()

def determine_phase_from_status(status_output):
    """
    Determine the current phase based on drone status
    """
    if "armed: true" in status_output and "mode: 'OFFBOARD'" in status_output:
        return "Scanning"
    elif "armed: true" in status_output:
        return "Takeoff"
    elif "landed" in status_output:
        return "Landing"
    else:
        return "Preflight"

# Missing function: monitor_scan_progress
def monitor_scan_progress():
    """
    Monitor scan progress and update phases accordingly
    """
    def progress_monitor():
        scan_started = False
        scanning_complete = False
        
        while True:
            try:
                # Check if command file indicates START
                if os.path.exists("command.json"):
                    with open("command.json", "r") as f:
                        data = json.load(f)
                        command = data.get("command", "STOP")
                        
                        if command == "START" and not scan_started:
                            scan_started = True
                            update_phase("Takeoff")
                            # Wait a bit then move to Scanning
                            time.sleep(3)
                            update_phase("Scanning")
                            
                        elif command == "STOP" and scan_started:
                            if not scanning_complete:
                                scanning_complete = True
                                update_phase("Landing")
                                time.sleep(2)
                                update_phase("Meshing")
                                time.sleep(5)
                                update_phase("Slicing")
                                time.sleep(5)
                                update_phase("Printing")
                                scan_started = False
                                scanning_complete = False
                        
            except Exception as e:
                print(f"Error monitoring scan progress: {e}")
            
            time.sleep(1)
    
    threading.Thread(target=progress_monitor, daemon=True).start()

# Update your send_command handler to increment scan number
@socketio.on("send_command")
def handle_command(data):
    command = data.get("command", "STOP")
    size = data.get("size", "MED")
    radius = SIZE_TO_RADIUS.get(size, 2.0)
    
    # If starting a new scan, increment the scan number
    if command == "START":
        new_scan = increment_scan_number()
        print(f"Starting new scan: {new_scan}")
        update_phase("Takeoff")
        
        # Send commands to drone
        send_radius(radius)
        send_ready()
        
    elif command == "STOP":
        print("Stopping scan")
        update_phase("Landing")
    
    print(f"Received command: {command}, size: {size}, radius: {radius}")
    with open("command.json", "w") as f:
        json.dump({"command": command, "size": size, "radius": radius}, f)
    emit("command_response", {"status": "ok"})

# --- Safe landing support ---
def land_drone():
    """Request a safe landing via MAVROS by switching mode to AUTO.LAND.
    If mavros isn't available on this host, this will simply no-op.
    """
    try:
        subprocess.run([
            "ros2", "service", "call",
            "/mavros/set_mode", "mavros_msgs/srv/SetMode",
            "{custom_mode: 'AUTO.LAND'}"
        ], check=False)
    except Exception as e:
        print(f"land_drone: failed to call MAVROS set_mode AUTO.LAND: {e}")


@socketio.on("land_flight")
def socket_land_flight():
    print("SocketIO: land_flight requested")
    update_phase("Landing")
    land_drone()
    emit("command_response", {"status": "ok", "action": "land"})


@socketio.on("emergency_stop")
def socket_emergency_stop():
    print("SocketIO: emergency_stop requested -> initiating landing")
    update_phase("Landing")
    land_drone()
    emit("command_response", {"status": "ok", "action": "emergency_land"})
