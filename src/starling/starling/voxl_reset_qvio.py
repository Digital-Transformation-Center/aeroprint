import subprocess

class VOXLQVIOController():
    def __init__(self) -> None:
        None

    def reset(self):
        try:
            subprocess.run(["voxl-reset-qvio"])
            return True
        except Exception as e:
            print(f"Error sending VIO reset command: {e}")
            return False