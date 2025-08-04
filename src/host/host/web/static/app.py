from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import os, json, subprocess, platform
import threading
import time
import struct
import random
from flask import jsonify

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

def get_latest_scan_number():
    """
    Get the latest/current scan number from the file system or a counter file
    """
    try:
        # Option 1: Read from a scan counter file
        if os.path.exists("current_scan.json"):
            with open("current_scan.json", "r") as f:
                data = json.load(f)
                return data.get("current_scan", 1)
        
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
    
    return new_scan

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
            from flask import send_file
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

# Add this route to increment scan number when starting a new scan
@app.route("/api/start_new_scan")
def start_new_scan():
    """
    Start a new scan (increments the scan number)
    """
    new_scan = increment_scan_number()
    return {"new_scan_num": new_scan, "message": f"Started scan {new_scan}"}

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

def send_pointcloud(points):
    # points should be a list of tuples/lists with (x, y, z) coordinates
    socketio.emit('pointcloud_point', {'points': points})

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

# Modified command handler to potentially start new scan
@socketio.on("send_command")
def handle_command(data):
    command = data.get("command", "STOP")
    size = data.get("size", "MED")
    radius = SIZE_TO_RADIUS.get(size, 2.0)
    
    # If starting a new scan, increment the scan number
    if command == "START":
        new_scan = increment_scan_number()
        print(f"Starting new scan: {new_scan}")
    
    print(f"Received command: {command}, size: {size}, radius: {radius}")
    with open("command.json", "w") as f:
        json.dump({"command": command, "size": size, "radius": radius}, f)
    emit("command_response", {"status": "ok"})

if __name__ == "__main__":
    socketio.run(app, debug=True)
