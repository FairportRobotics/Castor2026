import math
from phoenix6.hardware.pigeon2 import Pigeon2

class Pigeon:

    gyro: Pigeon2

    def setup(self):
        self.gyro.reset()

    def execute(self):
        pass

    def yaw(self) -> float:
        return self.gyro.get_yaw().value
    
    def roll(self) -> float:
        return self.gyro.get_roll().value

    def pitch(self) -> float:
        return self.gyro.get_pitch().value
    
    def rotation2d(self):
        return self.gyro.getRotation2d()
    
    def degrees_per_second_ccw(self) -> float:
        return self.gyro.get_angular_velocity_z_world().value

    def getRadiansPerSecCCW(self) -> float:
        return math.radians(self.degrees_per_second_ccw())
    
    def set_yaw(self, angle):
        self.gyro.set_yaw(angle)