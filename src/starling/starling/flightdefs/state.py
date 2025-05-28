
class State:
    def __init__(self):
        # Position
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # Velocity
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        # Acceleration
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0

        # Yaw (direction) and yaw rate
        self.yaw = 0.0
        self.yaw_rate = 0.0