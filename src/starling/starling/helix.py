from starling.flightdefs.path import Path
from starling.flightdefs.state import State
from math import pi, cos, sin, atan2
class Helix(Path):
    def __init__(self, rate=20):
        self.rate = rate

    def set_params(self, helix_height, helix_radius, helix_gap):
        self.helix_param_h = helix_height
        self.helix_param_r = helix_radius
        self.helix_param_g = helix_gap
        self.radial_multiplier = 30 # 


    def get_state(self, step):
        """
        This will give you a point containing state information
        """
        state = State()
        t = self.t(step)

        theta = self.theta(t)

        state.x = self.x(theta)
        state.y = self.y(theta)
        state.z = self.z(theta)
        state.vx = self.vx(theta)
        state.vy = self.vy(theta)
        state.vz = self.vz(theta)
        state.ax = self.ax(theta)
        state.ay = self.ay(theta)
        state.az = self.az(theta)

        state.yaw = self.yaw(step)
        
        state.yaw_rate = self.yaw_rate(step)

        return state
    
    def t(self, step):
        return step / self.rate

    def theta(self, t):
        return (2 * pi * t) / (self.radial_multiplier * self.helix_param_r)
    
    def x(self, theta):
        return self.helix_param_r * cos(theta)

    def y(self, theta):
        return self.helix_param_r * sin(theta)

    def z(self, theta):
        return self.helix_param_g * theta / (2 * pi)

    def vx(self, theta):
        return -self.helix_param_r * sin(theta)

    def vy(self, theta):
        return self.helix_param_r * cos(theta)

    def vz(self, theta):
        return self.helix_param_g / (2 * pi)
    
    def ax(self, theta):
        return -self.helix_param_r * cos(theta)
    
    def ay(self, theta):
        return -self.helix_param_r * sin(theta)
    
    def az(self, theta):
        return 0
    
    def yaw(self, step):
        theta = self.theta(self.t(step))
        return atan2(self.ay(theta), self.ax(theta))
    
    def yaw_rate(self, step):
        theta_next = self.theta(self.t(step + 1))
        theta = self.theta(self.t(step))
        return (theta_next - theta) * self.rate

    def get_duration(self):
        return 30 * self.helix_param_r * self.helix_param_h / (2 * self.helix_param_g)
    
    def get_num_steps(self):
        return int(self.get_duration() * self.rate)
