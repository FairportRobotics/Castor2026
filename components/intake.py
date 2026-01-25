import magicbot
from phoenix6.hardware import TalonFX
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from phoenix6.signals import NeutralModeValue
from phoenix6.controls import VoltageOut
from wpimath import units


class Intake:
    motor: TalonFX
    voltage = magicbot.will_reset_to(0.0)

    def execute(self):
        motor_control = VoltageOut(self.voltage)
        self.motor.set_control(motor_control)S

    def setup(self):
        config = TalonFXConfiguration()
        config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.motor.configurator.apply(config)

    def set_voltage(self, voltage: units.volts):
        self.voltage = voltage