"""
components/turret.py – TurretSubsystem ported to MagicBot component.

Hardware:
  - TalonFX   turret motor (Phoenix 6)
  - SparkMax   launcher motor (REV, brushless)
  - DigitalInput  limit switch
  - Servo         hood
"""

import enum
import math

import magicbot
import wpilib
from phoenix6.configs import (
    CurrentLimitsConfigs,
    FeedbackConfigs,
    MotorOutputConfigs,
    Slot0Configs,
    SoftwareLimitSwitchConfigs,
    TalonFXConfiguration,
)
from phoenix6.controls import PositionVoltage
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue
from rev import (
    ClosedLoopSlot,
    FeedbackSensor,
    PersistMode,
    ResetMode,
    SparkBase,
    SparkLowLevel,
    SparkMax,
    SparkMaxConfig,
)
from wpimath.geometry import Pose3d, Rotation2d

import constants


class TurretState(enum.Enum):
    INIT = enum.auto()
    HOMING = enum.auto()
    READY = enum.auto()


class TurretComponent:
    """MagicBot component that controls the turret, launcher, and hood."""

    def setup(self) -> None:
        # ── State ──────────────────────────────────────────────────────────
        self._turret_state = TurretState.INIT
        self._turret_offset_rotations: float = 0.0
        self._forward_limit: float = 0.0
        self._reverse_limit: float = 0.0
        self._launcher_setpoint_rpm: float = 0.0

        # ── Limit switch ───────────────────────────────────────────────────
        self._turret_limit_switch = wpilib.DigitalInput(
            constants.ShooterConstants.TURRET_LIMIT_CHANNEL
        )

        # ── Turret motor (TalonFX) ─────────────────────────────────────────
        self._turret_motor = TalonFX(constants.ShooterConstants.TURRET_MOTOR_ID)
        turret_cfg = TalonFXConfiguration()
        turret_cfg.slot0 = (
            Slot0Configs()
            .with_k_p(6)
            .with_k_i(0)
            .with_k_d(0.3)
            .with_k_s(0)
        )
        turret_cfg.motor_output = MotorOutputConfigs().with_inverted(
            InvertedValue.CLOCKWISE_POSITIVE
        )
        turret_cfg.current_limits = CurrentLimitsConfigs().with_stator_current_limit(50)
        turret_cfg.software_limit_switch = (
            SoftwareLimitSwitchConfigs()
            .with_forward_soft_limit_enable(False)
            .with_reverse_soft_limit_enable(False)
        )
        turret_cfg.feedback = FeedbackConfigs().with_feedback_rotor_offset(0)
        self._turret_motor.configurator.apply(turret_cfg)

        self._turret_position_signal = self._turret_motor.get_position()
        self._turret_error_signal = self._turret_motor.get_closed_loop_error()
        self._turret_control = PositionVoltage(0)

        # ── Launcher motor (SparkMax / REV NEO) ────────────────────────────
        self._launcher_motor = SparkMax(
            constants.ShooterConstants.LAUNCHER_MOTOR_ID,
            SparkLowLevel.MotorType.kBrushless,
        )
        launcher_cfg = SparkMaxConfig()
        # Apply equivalent of REV_NEO preset manually
        launcher_cfg.inverted(constants.ShooterConstants.LAUNCHER_MOTOR_INVERTED)
        launcher_cfg.voltageCompensation(10)
        launcher_cfg.closedLoop.P(0.0002).I(0.000001).D(0.0005)
        launcher_cfg.closedLoop.feedForward.kS(0.2).kV(0.0).kA(0.0)
        launcher_cfg.closedLoop.setFeedbackSensor(FeedbackSensor.kPrimaryEncoder)
        launcher_cfg.closedLoop.positionWrappingEnabled(False)
        self._launcher_motor.configure(
            launcher_cfg,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )
        self._launcher_controller = self._launcher_motor.getClosedLoopController()

        # ── Hood servo ─────────────────────────────────────────────────────
        self._hood = wpilib.Servo(constants.ShooterConstants.HOOD_SERVO_CHANNEL)

        self._turret_target_pose: Pose3d = Pose3d()

    # ── Public API ─────────────────────────────────────────────────────────────

    def set_launcher(self, speed_rpm: float) -> None:
        self._launcher_setpoint_rpm = speed_rpm
        if speed_rpm == 0:
            self._launcher_motor.stopMotor()
            return
        self._launcher_controller.setSetpoint(
            speed_rpm, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0
        )

    def is_launcher_up_to_speed(self) -> bool:
        # Currently always returns True (same as Java source)
        return True

    def set_target_elevation(self, elev: float) -> None:
        pos = abs((elev - 300) / 300)
        self._hood.set(pos)

    def home_turret(self) -> None:
        if self._turret_state != TurretState.INIT:
            return
        self._turret_motor.set(constants.ShooterConstants.HOMING_SPEED)
        self._turret_state = TurretState.HOMING

    def is_turret_ready(self) -> bool:
        return self._turret_state == TurretState.READY

    def get_turret_angle_robot_relative(self) -> float:
        return self._get_turret_motor_position() / constants.ShooterConstants.TURRET_GEAR_RATIO

    def set_turret_target_pose(self, target_pose: Pose3d) -> None:
        self._turret_target_pose = target_pose

    def get_turret_target_pose(self) -> Pose3d:
        return self._turret_target_pose

    def set_turret_field_relative(self, robot_pose, pos: float) -> None:
        """Set turret angle field-relative, accounting for robot heading."""
        if self._turret_state != TurretState.READY:
            return

        corrected_pos = -pos  # Flip angle to match CCW positive
        robot_rotations = robot_pose.rotation().toRotation2d().radians() / (2 * math.pi)
        turret_angle = (
            Rotation2d.fromRotations(corrected_pos)
            + Rotation2d.fromRotations(robot_rotations)
            - Rotation2d.fromRotations(0.5)
        )
        self.set_turret_robot_relative(turret_angle.radians() / (2 * math.pi))

    def set_turret_robot_relative(self, pos: float) -> None:
        if self._turret_state != TurretState.READY:
            return
        corrected_pos = -pos  # Flip angle to match CCW positive
        self.set_turret_motor_rotation(
            corrected_pos * constants.ShooterConstants.TURRET_GEAR_RATIO
        )

    def set_turret_motor_rotation(self, pos: float) -> None:
        pos_with_offset = pos + self._turret_offset_rotations
        self._turret_motor.set_control(self._turret_control.with_position(pos_with_offset))

    def is_turret_at_target(self) -> bool:
        return abs(self._turret_error_signal.refresh().value_as_double) <= 0.1

    # ── Feedback properties ────────────────────────────────────────────────────

    @magicbot.feedback
    def turret_state(self) -> str:
        return self._turret_state.name

    @magicbot.feedback
    def turret_angle_robot_relative(self) -> float:
        return self.get_turret_angle_robot_relative()

    @magicbot.feedback
    def turret_position_error(self) -> float:
        return self._turret_error_signal.refresh().value_as_double

    @magicbot.feedback
    def launcher_speed_rpm(self) -> float:
        return self._launcher_motor.getEncoder().getVelocity()

    @magicbot.feedback
    def turret_limit_switch(self) -> bool:
        return self._turret_limit_switch.get()

    # ── MagicBot lifecycle ─────────────────────────────────────────────────────

    def execute(self) -> None:
        # Homing state machine
        if self._turret_state == TurretState.HOMING:
            if self._turret_limit_switch.get():  # Switch tripped → homed
                self._turret_motor.stopMotor()
                raw_pos = self._turret_position_signal.refresh().value_as_double
                if wpilib.RobotBase.isReal():
                    self._turret_offset_rotations = raw_pos - 1.323
                else:
                    self._turret_offset_rotations = 0.0

                self._turret_state = TurretState.READY
                self._forward_limit = 1.5 + self._turret_offset_rotations
                self._reverse_limit = -1.5 + self._turret_offset_rotations

                # Enable software limits now that homing is complete
                limit_cfg = (
                    SoftwareLimitSwitchConfigs()
                    .with_forward_soft_limit_enable(True)
                    .with_forward_soft_limit_threshold(self._forward_limit)
                    .with_reverse_soft_limit_enable(True)
                    .with_reverse_soft_limit_threshold(self._reverse_limit)
                )
                self._turret_motor.configurator.apply(limit_cfg)

    # ── Helpers ────────────────────────────────────────────────────────────────

    def _get_turret_motor_position(self) -> float:
        return self._turret_position_signal.refresh().value_as_double - self._turret_offset_rotations
