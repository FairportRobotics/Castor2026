"""
components/swerve_module.py – Plain Python class (NOT a MagicBot component).

Four instances are owned by DriveComponent. Encapsulates all per-module hardware:
  - phoenix6 TalonFX  drive motor
  - phoenix6 TalonFX  steer motor
  - phoenix6 CANcoder
"""

import math

import ntcore
from phoenix6.configs import (
    CANcoderConfiguration,
    FeedbackConfigs,
    MagnetSensorConfigs,
    MotorOutputConfigs,
    Slot0Configs,
    TalonFXConfiguration,
)
from phoenix6.controls import PositionVoltage, VelocityVoltage
from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.signals import (
    FeedbackSensorSourceValue,
    InvertedValue,
    SensorDirectionValue,
)
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState

# Physical constants shared by all modules
GEAR_RATIO: float = 8.14
WHEEL_DIAMETER_METERS: float = 0.1016  # 4 inches
WHEEL_CIRCUMFERENCE_METERS: float = WHEEL_DIAMETER_METERS * 3.14159265358979


class SwerveModule:
    """
    Plain Python class (not a MagicBot component).
    Encapsulates a single swerve module: drive motor, steer motor, CANcoder.
    """

    def __init__(
        self,
        name: str,
        drive_id: int,
        steer_id: int,
        encoder_id: int,
        steer_offset_rotations: float,
        drive_inverted: bool,
        canbus: str = "Drive",
    ) -> None:
        self._name = name

        # ── CANcoder ───────────────────────────────────────────────────────
        self._encoder = CANcoder(encoder_id, canbus)
        encoder_cfg = CANcoderConfiguration()
        encoder_cfg.magnet_sensor = (
            MagnetSensorConfigs()
            .with_magnet_offset(steer_offset_rotations)
            .with_sensor_direction(SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE)
        )
        self._encoder.configurator.apply(encoder_cfg)

        # ── Drive motor (TalonFX) ──────────────────────────────────────────
        self._drive_motor = TalonFX(drive_id, canbus)
        drive_cfg = TalonFXConfiguration()
        drive_cfg.slot0 = (
            Slot0Configs()
            .with_k_p(0.1)
            .with_k_i(0.0)
            .with_k_d(0.01)
            .with_k_v(0.2)
        )
        drive_cfg.motor_output = MotorOutputConfigs().with_inverted(
            InvertedValue.CLOCKWISE_POSITIVE if drive_inverted else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        self._drive_motor.configurator.apply(drive_cfg)
        self._drive_velocity_signal = self._drive_motor.get_velocity()
        self._drive_position_signal = self._drive_motor.get_position()

        # ── Steer motor (TalonFX) ──────────────────────────────────────────
        self._steer_motor = TalonFX(steer_id, canbus)
        steer_cfg = TalonFXConfiguration()
        steer_cfg.slot0 = (
            Slot0Configs()
            .with_k_p(20)
            .with_k_i(0)
            .with_k_d(0.0)
            .with_k_v(0)
        )
        # Use the CANcoder as the absolute feedback source for the steer motor
        steer_cfg.feedback = (
            FeedbackConfigs()
            .with_feedback_remote_sensor_id(encoder_id)
            .with_feedback_sensor_source(FeedbackSensorSourceValue.REMOTE_CANCODER)
        )
        self._steer_motor.configurator.apply(steer_cfg)
        self._steer_position_signal = self._steer_motor.get_position()

        # Controls
        self._drive_control = VelocityVoltage(0)
        self._steer_control = PositionVoltage(0)

        # Desired state (for telemetry)
        self._desired_state = SwerveModuleState(0, Rotation2d())

        # ── NetworkTables telemetry entries ────────────────────────────────
        inst = ntcore.NetworkTableInstance.getDefault()
        tbl = inst.getTable(f"SwerveModules/{name}")
        self._nt_desired_speed = tbl.getDoubleTopic("desired_speed_mps").publish()
        self._nt_desired_angle = tbl.getDoubleTopic("desired_angle_deg").publish()
        self._nt_actual_speed  = tbl.getDoubleTopic("actual_speed_mps").publish()
        self._nt_actual_angle  = tbl.getDoubleTopic("actual_angle_deg").publish()

    # ── Public API ─────────────────────────────────────────────────────────────

    def set_desired_state(self, state: SwerveModuleState) -> None:
        """Optimize the state and apply drive velocity + steer position controls."""
        state.optimize(self._get_steer_angle())  # mutates in-place; returns None in WPILib 2026
        self._desired_state = state

        # Drive: convert m/s → rotations/s
        drive_rps = (state.speed / WHEEL_CIRCUMFERENCE_METERS) * GEAR_RATIO
        self._drive_motor.set_control(self._drive_control.with_velocity(drive_rps))

        # Steer: position in rotations (CANcoder feedback is in rotations)
        steer_rotations = state.angle.radians() / (2 * math.pi)
        self._steer_motor.set_control(self._steer_control.with_position(steer_rotations))

    def get_state(self) -> SwerveModuleState:
        """Return current speed (m/s) and angle from motor signals."""
        velocity_rps = self._drive_velocity_signal.refresh().value_as_double
        speed_mps = (velocity_rps / GEAR_RATIO) * WHEEL_CIRCUMFERENCE_METERS
        return SwerveModuleState(speed_mps, self._get_steer_angle())

    def get_position(self) -> SwerveModulePosition:
        """Return current distance (m) and angle for odometry update."""
        position_rotations = self._drive_position_signal.refresh().value_as_double
        distance_meters = (position_rotations / GEAR_RATIO) * WHEEL_CIRCUMFERENCE_METERS
        return SwerveModulePosition(distance_meters, self._get_steer_angle())

    def publish_telemetry(self) -> None:
        """Write module telemetry to NetworkTables. Called each loop by DriveComponent."""
        actual = self.get_state()
        self._nt_desired_speed.set(self._desired_state.speed)
        self._nt_desired_angle.set(self._desired_state.angle.degrees())
        self._nt_actual_speed.set(actual.speed)
        self._nt_actual_angle.set(actual.angle.degrees())

    # ── Helpers ────────────────────────────────────────────────────────────────

    def _get_steer_angle(self) -> Rotation2d:
        rotations = self._steer_position_signal.refresh().value_as_double
        return Rotation2d.fromRotations(rotations)
