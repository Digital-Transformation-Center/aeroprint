from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import os, json, subprocess, platform

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

@socketio.on("send_command")
def handle_command(data):
    command = data.get("command", "STOP")
    size = data.get("size", "MED")
    radius = SIZE_TO_RADIUS.get(size, 2.0)
    print(f"Received command: {command}, size: {size}, radius: {radius}")  # <-- Add this line
    with open("command.json", "w") as f:
        json.dump({"command": command, "size": size, "radius": radius}, f)
    emit("command_response", {"status": "ok"})

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


if __name__ == "__main__":
    socketio.run(app, debug=True)
