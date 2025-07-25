from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import os, json, subprocess, paramiko

app = Flask(__name__)
socketio = SocketIO(app)

# Size → Flight parameter mapping
SIZE_TO_RADIUS = {
    "SM": 1.0,
    "MED": 2.0,
    "LG": 3.5
}

import platform

def is_starling_reachable():
    try:
        param = "-n" if platform.system().lower() == "windows" else "-c"
        result = subprocess.run(
            ["ping", param, "1", "m0054"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        return result.returncode == 0
    except:
        return False
    host = "m0054"
    username = "root"
    password = os.environ.get("STARLING_SSH_PASSWORD")
    launch_command = "source ~/aeroprint/install/setup.bash && ros2 launch starling starling_launch.py"
    username = "root"
    password = "oelinux123"
    launch_command = "source ~/aeroprint/install/setup.bash && ros2 launch starling starling_launch.py"

    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(host, username=username, password=password)
        stdin, stdout, stderr = ssh.exec_command(launch_command)
        print(stdout.read().decode())
        print(stderr.read().decode())
        ssh.close()
        return True
    except Exception as e:
        print(f"SSH error: {e}")
        return False

@app.route("/")
def home():
    selected_size = "MED"
    command = "STOP"
    stage = "Preflight"
    starling_status = is_starling_reachable()

    if os.path.exists("command.json") and os.path.getsize("command.json") > 0:
        try:
            with open("command.json") as f:
                data = json.load(f)
                selected_size = data.get("size", selected_size)
                command = data.get("command", command)
        except:
            pass

    return render_template("index.html",
        selected_size=selected_size,
        command=command,
        stage=stage,
        starling=starling_status
    )

@socketio.on("send_command")
def handle_command(data):
    print("Received command:", data)  # Add this line
    command = data.get("command", "STOP")
    size = data.get("size", "MED")

    if not is_starling_reachable():
        emit("command_response", {"status": "error", "message": "Starling is unreachable"})
        return

    # Save to command.json
    try:
        with open("command.json", "w") as f:
            json.dump({"command": command, "size": size}, f)

        # Also save radius to a flight_params.json
        radius = SIZE_TO_RADIUS.get(size, 2.0)
        with open("flight_params.json", "w") as f:
            json.dump({"radius": radius}, f)

        # Optional SSH launch if START
        if command == "START":
            launched = launch_starling_via_ssh()
            if not launched:
                emit("command_response", {"status": "error", "message": "Launch failed"})
                return

        emit("command_response", {"status": "ok", "command": command, "size": size})
    except Exception as e:
        emit("command_response", {"status": "error", "message": str(e)})

def update_phase(new_phase):
    # Update status.json
    with open("status.json", "w") as f:
        json.dump({"stage": new_phase}, f)
    # Emit to all connected clients
    socketio.emit('status_update', {'phase': new_phase})

# Example usage: call update_phase("Processing") when phase changes

if __name__ == "__main__":
    socketio.run(app, debug=True)
