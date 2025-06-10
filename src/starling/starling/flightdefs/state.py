
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

    def get_position(self, invert_axis=None):
        if invert_axis is not None:
            if invert_axis == 'x':
                return [-self.x, self.y, self.z]
            elif invert_axis == 'y':
                return [self.x, -self.y, self.z]
            elif invert_axis == 'z':
                return [self.x, self.y, -self.z]
        return [self.x, self.y, self.z]
    def get_velocity(self, invert_axis=None):
        if invert_axis is not None:
            if invert_axis == 'x':
                return [-self.vx, self.vy, self.vz]
            elif invert_axis == 'y':
                return [self.vx, -self.vy, self.vz]
            elif invert_axis == 'z':
                return [self.vx, self.vy, -self.vz]
        return [self.vx, self.vy, self.vz]
    def get_acceleration(self, invert_axis=None):
        if invert_axis is not None:
            if invert_axis == 'x':
                return [-self.ax, self.ay, self.az]
            elif invert_axis == 'y':
                return [self.ax, -self.ay, self.az]
            elif invert_axis == 'z':
                return [self.ax, self.ay, -self.az]
        return [self.ax, self.ay, self.az]
    def get_yaw(self):
        return self.yaw
    def get_yaw_rate(self):
        return self.yaw_rate
    
class Position:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)