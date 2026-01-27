import math

class SwerveModule:
    name: str
    drive_id: int
    drive_P: float
    drive_I: float
    drive_D: float
    turning_id: int
    turning_P: float
    turning_I: float
    turning_D: float
    encoder_id: int
    gear_ratio: float
    wheel_diameter: float    
    
    def setup(self):
        self.max_angular_velocity = math.pi
        self.max_angular_acceleration = math.tau

    def execute(self):
        pass