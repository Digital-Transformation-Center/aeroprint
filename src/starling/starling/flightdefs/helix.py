from starling.flightdefs.path import Path
from starling.flightdefs.state import State, Position
from math import pi, cos, sin, atan2
class Helix(Path):
    def __init__(self, rate=20):
        self.rate = rate

    def set_params(self, helix_height, helix_radius, helix_num_passes, helix_start_height, helix_rate=30):
        self.helix_param_h = helix_height
        self.helix_param_r = helix_radius
        self.helix_param_p = helix_num_passes
        self.helix_param_g = helix_height / helix_num_passes
        self.helix_param_sh = helix_start_height
        if self.helix_param_sh < 0.1:
            self.helix_param_sh = 0.1  # Ensure starting height is not too low
        self.helix_param_rate = helix_rate

        self.helix_starting_position = Position(0, 0, self.helix_param_sh + self.helix_param_h)  # Starting position of the helix
        self.helix_ending_position = Position(0, 0, self.helix_param_sh)  # Ending position of the helix

        self.theta_start = 0
        self.theta_end = 2 * pi * self.helix_param_p  # Total angle covered by the helix
        self.num_steps = helix_num_passes * helix_rate * self.rate
        self.steps_per_pass = helix_rate * self.rate  # Steps per complete turn of the helix
        self.dadt = 1.0 / self.rate # The time difference between each step in seconds

    def get_state(self, step):
        """
        This will give you a point containing state information
        """
        if step < 0 or step > self.num_steps:
            raise ValueError("Step out of range for given helix parameters.")
        
        state = State()
        theta = self._theta(step)

        state.x = self._x(theta)
        state.y = self._y(theta)
        state.z = self._z(theta)
        state.vx = self._vx(theta)
        state.vy = self._vy(theta)
        state.vz = self._vz(theta)
        state.ax = self._ax(theta)
        state.ay = self._ay(theta)
        state.az = self._az(theta)

        state.yaw = self._yaw(step)
        state.yaw_rate = self._yaw_rate(step)

        return state
    
    def _t(self, step):
        return step / self.rate

    def _theta(self, step):
        return 2 * pi * step / self.steps_per_pass

    def _x(self, theta):
        return self.helix_param_r - self.helix_param_r * cos(theta)

    def _y(self, theta):
        return self.helix_param_r * sin(theta)

    def _z(self, theta):
        return self.helix_starting_position.z - self.helix_param_g * theta / (2 * pi)

    def _vx(self, theta):
        return self.helix_param_r * sin(theta) * self.dadt

    def _vy(self, theta):
        return self.helix_param_r * cos(theta) * self.dadt

    def _vz(self, theta):
        return (- self.helix_param_h / self.get_duration()) * self.dadt
    
    def _ax(self, theta):
        return self.helix_param_r * cos(theta) * self.dadt

    def _ay(self, theta):
        return -self.helix_param_r * sin(theta) * self.dadt

    def _az(self, theta):
        return 0

    def _yaw(self, step):
        theta = self._theta(step)
        return atan2(self._ay(theta), self._ax(theta))

    def _yaw_rate(self, step):
        if step == 0:
            return 0
        # Based on how px4 deals with yaw: we may get a jump at pi.
        if self._yaw(step) - self._yaw(step - 1) > pi/2:
            return (self._yaw(step + 1) - self._yaw(step)) * self.rate
        return (self._yaw(step) - self._yaw(step - 1)) * self.rate

    def get_duration(self):
        return self.num_steps / self.rate
    
    def get_num_steps(self):
        return self.num_steps
    
    def get_duration(self):
        return self.num_steps / self.rate

    def get_starting_position(self, invert_axis=None):
        if invert_axis is not None:
            if invert_axis == 'x':
                return Position(-self.helix_starting_position.x, self.helix_starting_position.y, self.helix_starting_position.z)
            elif invert_axis == 'y':
                return Position(self.helix_starting_position.x, -self.helix_starting_position.y, self.helix_starting_position.z)
            elif invert_axis == 'z':
                return Position(self.helix_starting_position.x, self.helix_starting_position.y, -self.helix_starting_position.z)
        return self.helix_starting_position

    def get_ending_position(self, invert_axis=None):
        if invert_axis is not None:
            if invert_axis == 'x':
                return Position(-self.helix_ending_position.x, self.helix_ending_position.y, self.helix_ending_position.z)
            elif invert_axis == 'y':
                return Position(self.helix_ending_position.x, -self.helix_ending_position.y, self.helix_ending_position.z)
            elif invert_axis == 'z':
                return Position(self.helix_ending_position.x, self.helix_ending_position.y, -self.helix_ending_position.z)
        return self.helix_ending_position

