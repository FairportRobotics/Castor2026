"""
constants.py – robot-wide constants for Castor 2026 (Python/MagicBot port).

All Java "public static final" fields become Python class-level attributes.
"""

import enum
import math

from phoenix6.signals import InvertedValue
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from wpimath.geometry import (
    Pose3d,
    Rotation3d,
    Transform3d,
    Translation2d,
    Translation3d,
)


class OperatorConstants:
    CONTROLLER_PORT: int = 0


class CameraConstants:

    class IDFilters:
        RED_HUB_SHOOTING_IDS: list[int] = [8, 9, 10, 11]
        BLUE_HUB_SHOOTING_IDS: list[int] = [27, 26, 25, 24]

    BACK_CAMERA_NAME: str = "limelight-back"
    BACK_LOCALIZATION_PIPELINE_NUMBER: int = 0
    BACK_HUB_TRACKING_2D_PIPELINE_NUMBER: int = 1
    BACK_HUB_TRACKING_3D_PIPELINE_NUMBER: int = 2

    FRONT_CAMERA_NAME: str = "limelight-front"
    FRONT_LOCALIZATION_PIPELINE_NUMBER: int = 0

    BLUE_ZONE_COOR: list[float] = [0, 3.5]
    RED_ZONE_COOR: list[float] = [13, 16]
    NEUTRAL_ZONE_COOR: list[float] = [5.5, 11]
    NEUTRAL1_ZONE_COOR: list[float] = [5.5, 11, 0, 3]
    NEUTRAL2_ZONE_COOR: list[float] = [5.5, 11, 5, 8]
    BLUE1_ZONE_COOR: list[float] = [0, 3.5, 0, 3]
    BLUE2_ZONE_COOR: list[float] = [0, 3.5, 5, 8]
    RED1_ZONE_COOR: list[float] = [13, 16, 0, 3]
    RED2_ZONE_COOR: list[float] = [13, 16, 5, 8]


# Swerve encoder offsets as of 3/2/26
class SwerveDriveOffsets:
    FRONT_LEFT_MOTOR_OFFSET: float = -0.336426
    FRONT_RIGHT_MOTOR_OFFSET: float = 0.108887
    BACK_LEFT_MOTOR_OFFSET: float = -0.492432
    BACK_RIGHT_MOTOR_OFFSET: float = 0.470215


# CAN ID numbering convention:
#   1x = Front Left, 2x = Front Right, 3x = Back Left, 4x = Back Right
#   Single-digit IDs are non-motor devices (e.g. Pigeon2)
class SwerveDriveIDs:
    FRONT_LEFT_STEER_ID: int = 11
    FRONT_LEFT_DRIVE_ID: int = 12
    FRONT_LEFT_ENCODER_ID: int = 13

    FRONT_RIGHT_STEER_ID: int = 21
    FRONT_RIGHT_DRIVE_ID: int = 22
    FRONT_RIGHT_ENCODER_ID: int = 23

    BACK_LEFT_STEER_ID: int = 31
    BACK_LEFT_DRIVE_ID: int = 32
    BACK_LEFT_ENCODER_ID: int = 33

    BACK_RIGHT_STEER_ID: int = 41
    BACK_RIGHT_DRIVE_ID: int = 42
    BACK_RIGHT_ENCODER_ID: int = 43


class RobotChassisLimits:
    # Max linear velocity (m/s)
    MAX_ROBOT_LINEAR_VELOCITY: float = 3.0
    # Max linear acceleration (m/s²)
    MAX_ROBOT_LINEAR_VELOCITY_ACCEL: float = 2.0
    # Max angular velocity (rad/s)
    MAX_ROBOT_ANGULAR_VELOCITY: float = math.pi
    # Max angular acceleration (rad/s²)
    MAX_ROBOT_ANGULAR_VELOCITY_ACCEL: float = math.pi


class ExtraIDEntities:
    PIGEON_ID: int = 1


class IntakeConstants:
    DEPLOY_MOTOR_ID: int = 17
    DEPLOY_MOTOR_INVERTED: bool = True  # TODO: Test this

    INTAKE_MOTOR_ID: int = 2
    INTAKE_MOTOR_DIRECTION: InvertedValue = InvertedValue.CLOCKWISE_POSITIVE  # TODO: Test this


class ShooterConstants:
    LAUNCHER_MOTOR_ID: int = 3
    LAUNCHER_MOTOR_INVERTED: bool = False

    TURRET_MOTOR_ID: int = 4
    TURRET_MOTOR_DIRECTION: InvertedValue = InvertedValue.CLOCKWISE_POSITIVE
    TURRET_LIMIT_CHANNEL: int = 9

    # Plain floats (degrees) replacing edu.wpi.first.units.Degrees.of(x)
    LIMIT_AZIMUTH_POS: float = 165.0   # degrees
    LIMIT_AZIMUTH_NEG: float = 5.0     # degrees

    HOMING_SPEED: float = 0.05
    TURRET_GEAR_RATIO: float = 6.0

    HOOD_SERVO_CHANNEL: int = 0
    HOOD_SERVO_INVERTED: bool = False   # TODO: Test this
    HOOD_ENCODER_ID: int = 4

    TARGET_ELEVATION_DIF: float = 1.0   # degrees
    LIMIT_ELEVATION_POS: float = 90.0   # degrees
    LIMIT_ELEVATION_NEG: float = 0.0    # degrees

    DEFLECTOR_STORED_ANGLE: float = 0.0
    DEFLECTOR_SET_ANGLE1: float = 100.0
    DEFLECTOR_SET_ANGLE2: float = 200.0
    DEFLECTOR_SET_ANGLE3: float = 300.0
    DEFLECTOR_SERVO_RATIO: float = 1.0


class HopperConstants:
    KICKER_MOTOR_ID: int = 9
    KICKER_MOTOR_DIRECTION: InvertedValue = InvertedValue.CLOCKWISE_POSITIVE

    SPINDEXER_MOTOR_ID: int = 8
    SPINDEXER_MOTOR_DIRECTION: InvertedValue = InvertedValue.COUNTER_CLOCKWISE_POSITIVE


# ── Swerve module Translation2d locations ────────────────────────────────────
# Robot is 22" × 22"; modules are at the corners.
# Half of 22" = 11" = 0.2794 m from robot centre to each module.
# WPILib convention: +X is forward, +Y is to the left.
class SwerveDriveModuleLocations:
    _HALF: float = 0.2794  # metres (11 inches)

    FRONT_LEFT:  Translation2d = Translation2d( _HALF,  _HALF)
    FRONT_RIGHT: Translation2d = Translation2d( _HALF, -_HALF)
    BACK_LEFT:   Translation2d = Translation2d(-_HALF,  _HALF)
    BACK_RIGHT:  Translation2d = Translation2d(-_HALF, -_HALF)


# ── April-tag field layout (loaded once at import time) ───────────────────────
_field_tags = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltWelded)


class FieldPoses:
    BLUE_HUB_POSE: Pose3d = _field_tags.getTagPose(26).transformBy(
        Transform3d(Translation3d(-0.5207, 0, 0), Rotation3d())
    )
    RED_HUB_POSE: Pose3d = _field_tags.getTagPose(10).transformBy(
        Transform3d(Translation3d(-0.5207, 0, 0), Rotation3d())
    )

    # Passing targets — left/right from driver's perspective;
    # origin at blue side of field; rotations are zero (don't matter).
    BLUE_LEFT_ALLIANCE_ZONE_PASS_TARGET_POSE:  Pose3d = Pose3d(1.99,  6.35, 0, Rotation3d())
    BLUE_RIGHT_ALLIANCE_ZONE_PASS_TARGET_POSE: Pose3d = Pose3d(1.99,  1.72, 0, Rotation3d())
    BLUE_LEFT_NEUTRAL_ZONE_PASS_TARGET_POSE:   Pose3d = Pose3d(8.27,  6.35, 0, Rotation3d())
    BLUE_RIGHT_NEUTRAL_ZONE_PASS_TARGET_POSE:  Pose3d = Pose3d(8.27,  1.72, 0, Rotation3d())

    RED_LEFT_ALLIANCE_ZONE_PASS_TARGET_POSE:   Pose3d = Pose3d(14.55, 1.72, 0, Rotation3d())
    RED_RIGHT_ALLIANCE_ZONE_PASS_TARGET_POSE:  Pose3d = Pose3d(14.55, 6.35, 0, Rotation3d())
    RED_LEFT_NEUTRAL_ZONE_PASS_TARGET_POSE:    Pose3d = Pose3d(8.27,  1.72, 0, Rotation3d())
    RED_RIGHT_NEUTRAL_ZONE_PASS_TARGET_POSE:   Pose3d = Pose3d(8.27,  6.35, 0, Rotation3d())


# ── Shooting-region boundary lines ───────────────────────────────────────────
class ShootingRegionDimensions:
    BLUE_ALLIANCE_ZONE_REGION_X_MAX: float = 3.5
    RED_ALLIANCE_ZONE_REGION_X_MIN:  float = 13.0
    NEUTRAL_ZONE_REGION_X_MIN:       float = 5.5
    NEUTRAL_ZONE_REGION_X_MAX:       float = 11.0
    BLUE_RIGHT_REGION_Y_MAX:         float = 3.0
    RED_LEFT_REGION_Y_MAX:           float = 3.0
    BLUE_LEFT_REGION_Y_MIN:          float = 5.0
    RED_RIGHT_REGION_Y_MIN:          float = 5.0


# ── Shooting regions (Java enum → Python enum.Enum) ───────────────────────────
class ShootingRegion(enum.Enum):
    NON_SHOOTING_REGION               = enum.auto()
    BLUE_OWN_ALLIANCE_ZONE            = enum.auto()
    BLUE_LEFT_NEUTRAL_ZONE            = enum.auto()
    BLUE_RIGHT_NEUTRAL_ZONE           = enum.auto()
    BLUE_LEFT_OPPONENT_ALLIANCE_ZONE  = enum.auto()
    BLUE_RIGHT_OPPONENT_ALLIANCE_ZONE = enum.auto()
    RED_OWN_ALLIANCE_ZONE             = enum.auto()
    RED_LEFT_NEUTRAL_ZONE             = enum.auto()
    RED_RIGHT_NEUTRAL_ZONE            = enum.auto()
    RED_LEFT_OPPONENT_ALLIANCE_ZONE   = enum.auto()
    RED_RIGHT_OPPONENT_ALLIANCE_ZONE  = enum.auto()
