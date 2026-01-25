import math
from phoenix6.hardware.pigeon2 import Pigeon2
import navx


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

    def radians_per_second_ccw(self) -> float:
        return math.radians(self.degrees_per_second_ccw())

    def set_yaw(self, angle):
        self.gyro.set_yaw(angle)


class NavX:
    gyro: navx.AHRS  # Either navx.AHRS.create_spi() or navx.AHRS.create_i2c()

    def setup(self):
        self.gyro.reset()

    def execute(self):
        pass

    def yaw(self) -> float:
        return self.gyro.getYaw()

    def roll(self) -> float:
        return self.gyro.getRoll()

    def pitch(self) -> float:
        return self.gyro.getPitch()

    def is_calibrating(self) -> bool:
        return self.gyro.isCalibrating()

    def is_connected(self) -> bool:
        return self.gyro.isConnected()

    def rotation2d(self):
        return self.gyro.getRotation2d()

    def rotation3d(self):
        return self.gyro.getRotation3d()

    def reset(self) -> None:
        self.gyro.reset()

    def fused_heading(self) -> float:
        return self.gyro.getFusedHeading()

    def zero_yaw(self) -> None:
        self.gyro.zeroYaw()
