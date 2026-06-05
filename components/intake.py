"""
components/intake.py – IntakeSubsystem ported to MagicBot component.

Hardware:
  - TalonFX         intake motor (Phoenix 6)
  - WPI_TalonSRX    deploy motor (Phoenix 5) with forward/reverse limit switches
"""

import wpilib
from phoenix5 import LimitSwitchNormal, LimitSwitchSource, WPI_TalonSRX
from phoenix6.configs import MotorOutputConfigs, TalonFXConfiguration
from phoenix6.hardware import TalonFX

import constants


class IntakeComponent:
    """MagicBot component that controls the intake and deploy mechanism."""

    def setup(self) -> None:
        # ── Intake motor (Phoenix 6 TalonFX) ───────────────────────────────
        self._intake_motor = TalonFX(constants.IntakeConstants.INTAKE_MOTOR_ID)
        intake_cfg = TalonFXConfiguration()
        intake_cfg.motor_output = MotorOutputConfigs().with_inverted(
            constants.IntakeConstants.INTAKE_MOTOR_DIRECTION
        )
        self._intake_motor.configurator.apply(intake_cfg)

        # ── Deploy motor (Phoenix 5 WPI_TalonSRX) ──────────────────────────
        self._deploy_motor = WPI_TalonSRX(constants.IntakeConstants.DEPLOY_MOTOR_ID)
        self._deploy_motor.configForwardLimitSwitchSource(
            LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen
        )
        self._deploy_motor.configReverseLimitSwitchSource(
            LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen
        )
        self._deploy_motor.setInverted(constants.IntakeConstants.DEPLOY_MOTOR_INVERTED)

        # Controller reference — injected by robot.py after setup
        self._controller: wpilib.XboxController | None = None

    # ── Public API ─────────────────────────────────────────────────────────────

    def set_controller(self, controller: wpilib.XboxController) -> None:
        """Inject the driver Xbox controller for rumble feedback."""
        self._controller = controller

    def deploy(self) -> None:
        """Run the deploy motor forward briefly (call once; timing handled externally)."""
        self._deploy_motor.set(1.0)

    def stop_deploy(self) -> None:
        self._deploy_motor.stopMotor()

    def reset_deploy(self) -> None:
        """Retract the intake deploy mechanism."""
        self._deploy_motor.set(-1.0)

    def set_speed(self, speed: float) -> None:
        self._intake_motor.set(speed)

    def stop_intake(self) -> None:
        self._intake_motor.stopMotor()
        if self._controller is not None:
            self._controller.setRumble(wpilib.XboxController.RumbleType.kBothRumble, 0.0)

    def intake(self) -> None:
        """Toggle intake on/off with controller rumble feedback."""
        if self._intake_motor.get() == 0:
            self.set_speed(-0.5)
            if self._controller is not None:
                self._controller.setRumble(wpilib.XboxController.RumbleType.kBothRumble, 0.5)
        else:
            self._intake_motor.stopMotor()
            if self._controller is not None:
                self._controller.setRumble(wpilib.XboxController.RumbleType.kBothRumble, 0.0)

    def reverse_intake(self) -> None:
        self.set_speed(0.5)

    # ── MagicBot lifecycle ─────────────────────────────────────────────────────

    def execute(self) -> None:
        pass
