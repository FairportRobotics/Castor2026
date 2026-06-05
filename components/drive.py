"""
components/drive.py – DriveSubsystem ported to MagicBot component.

Owns four SwerveModule instances.  All swerve math uses standard wpimath.
Integrates with PathPlannerLib and Limelight vision.
"""

import math
import os

import magicbot
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import PIDConstants, RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.logging import PathPlannerLogging
from phoenix6.hardware import Pigeon2
from wpilib import DriverStation, RobotState, SendableChooser, SmartDashboard, getDeployDirectory
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Pose3d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
)

import constants
import LimelightHelpers
from components.swerve_module import SwerveModule


class DriveComponent:
    """
    MagicBot component that drives four swerve modules and fuses vision
    measurements via a SwerveDrive4PoseEstimator.
    """

    def setup(self) -> None:
        # ── Swerve modules ─────────────────────────────────────────────────
        self._front_left = SwerveModule(
            name="Front Left",
            drive_id=constants.SwerveDriveIDs.FRONT_LEFT_DRIVE_ID,
            steer_id=constants.SwerveDriveIDs.FRONT_LEFT_STEER_ID,
            encoder_id=constants.SwerveDriveIDs.FRONT_LEFT_ENCODER_ID,
            steer_offset_rotations=constants.SwerveDriveOffsets.FRONT_LEFT_MOTOR_OFFSET,
            drive_inverted=False,
            canbus="Drive",
        )
        self._front_right = SwerveModule(
            name="Front Right",
            drive_id=constants.SwerveDriveIDs.FRONT_RIGHT_DRIVE_ID,
            steer_id=constants.SwerveDriveIDs.FRONT_RIGHT_STEER_ID,
            encoder_id=constants.SwerveDriveIDs.FRONT_RIGHT_ENCODER_ID,
            steer_offset_rotations=constants.SwerveDriveOffsets.FRONT_RIGHT_MOTOR_OFFSET,
            drive_inverted=True,   # Front Right is inverted
            canbus="Drive",
        )
        self._back_left = SwerveModule(
            name="Back Left",
            drive_id=constants.SwerveDriveIDs.BACK_LEFT_DRIVE_ID,
            steer_id=constants.SwerveDriveIDs.BACK_LEFT_STEER_ID,
            encoder_id=constants.SwerveDriveIDs.BACK_LEFT_ENCODER_ID,
            steer_offset_rotations=constants.SwerveDriveOffsets.BACK_LEFT_MOTOR_OFFSET,
            drive_inverted=False,
            canbus="Drive",
        )
        self._back_right = SwerveModule(
            name="Back Right",
            drive_id=constants.SwerveDriveIDs.BACK_RIGHT_DRIVE_ID,
            steer_id=constants.SwerveDriveIDs.BACK_RIGHT_STEER_ID,
            encoder_id=constants.SwerveDriveIDs.BACK_RIGHT_ENCODER_ID,
            steer_offset_rotations=constants.SwerveDriveOffsets.BACK_RIGHT_MOTOR_OFFSET,
            drive_inverted=True,   # Back Right is inverted
            canbus="Drive",
        )
        self._modules = [
            self._front_left,
            self._front_right,
            self._back_left,
            self._back_right,
        ]

        # ── Kinematics ─────────────────────────────────────────────────────
        self._kinematics = SwerveDrive4Kinematics(
            constants.SwerveDriveModuleLocations.FRONT_LEFT,
            constants.SwerveDriveModuleLocations.FRONT_RIGHT,
            constants.SwerveDriveModuleLocations.BACK_LEFT,
            constants.SwerveDriveModuleLocations.BACK_RIGHT,
        )

        # ── Gyro ───────────────────────────────────────────────────────────
        self._pigeon = Pigeon2(constants.ExtraIDEntities.PIGEON_ID, "Drive")

        # ── Pose estimator ─────────────────────────────────────────────────
        self._pose_estimator = SwerveDrive4PoseEstimator(
            self._kinematics,
            self._pigeon.getRotation2d(),
            self._module_positions(),
            Pose2d(),
        )
        # 3 std-dev values: (x_std, y_std, heading_std)
        self._pose_estimator.setVisionMeasurementStdDevs((0.7, 0.7, 0.1))

        # ── Desired chassis speeds (reset each loop) ────────────────────────
        self._desired_speeds = ChassisSpeeds()

        # ── PathPlanner target pose (for @feedback) ────────────────────────
        self._pp_target_pose = Pose2d()

        # ── PathPlanner / AutoBuilder ──────────────────────────────────────
        # _DriveSubsystemAdapter is imported from robot.py at runtime to
        # avoid circular imports; it is passed in via inject_drive_adapter().
        self._drive_adapter = None
        self._auto_chooser: SendableChooser | None = None

    def inject_drive_adapter(self, adapter, robot_config: RobotConfig) -> None:
        """
        Called by robot.py after all objects are constructed.
        Finalises AutoBuilder configuration and builds the auto chooser.
        """
        AutoBuilder.configure(
            self.get_robot_pose_2d,                   # pose supplier
            self.set_pose,                            # pose resetter
            self.get_robot_relative_speeds,           # speeds supplier
            lambda speeds, ff: self.set_chassis_speeds(speeds),  # consumer (no negation)
            PPHolonomicDriveController(
                PIDConstants(5, 0, 0),
                PIDConstants(math.pi, 0, 0),
            ),
            robot_config,
            self._is_red_alliance,                    # flip-path predicate
            adapter,                                  # dummy Subsystem
        )

        # Build auto chooser manually so a bad path file skips that auto instead
        # of crashing the entire chooser (PathPlanner throws ZeroDivisionError on
        # paths with consecutive rotation targets at the same point).
        autos_dir = os.path.join(getDeployDirectory(), "pathplanner", "autos")
        self._auto_chooser = SendableChooser()
        self._auto_chooser.setDefaultOption("Do Nothing", None)
        if os.path.isdir(autos_dir):
            for filename in os.listdir(autos_dir):
                auto_name = filename.removesuffix(".auto")
                try:
                    self._auto_chooser.addOption(auto_name, AutoBuilder.buildAuto(auto_name))
                except Exception as build_err:
                    SmartDashboard.putString(f"Auto/{auto_name} error", str(build_err))
        SmartDashboard.putData("Auto Chooser", self._auto_chooser)

        PathPlannerLogging.setLogTargetPoseCallback(
            lambda pose: setattr(self, "_pp_target_pose", pose)
        )

    # ── Public drive API ───────────────────────────────────────────────────────

    def set_chassis_speeds_field_relative(
        self, vx: float, vy: float, omega: float
    ) -> None:
        """
        Convert joystick inputs (with quadratic scaling) to robot-relative
        ChassisSpeeds and store them to be applied in execute().
        """
        scaled_vx = vx * abs(vx) * constants.RobotChassisLimits.MAX_ROBOT_LINEAR_VELOCITY
        scaled_vy = vy * abs(vy) * constants.RobotChassisLimits.MAX_ROBOT_LINEAR_VELOCITY
        scaled_omega = omega * abs(omega) * constants.RobotChassisLimits.MAX_ROBOT_ANGULAR_VELOCITY

        self._desired_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            scaled_vx,
            scaled_vy,
            scaled_omega,
            self._pigeon.getRotation2d(),
        )

    def set_chassis_speeds(self, speeds: ChassisSpeeds) -> None:
        """Direct robot-relative speed input (used by PathPlanner consumer)."""
        self._desired_speeds = speeds

    def stop_drive(self) -> None:
        self._desired_speeds = ChassisSpeeds()

    def rotate_chassis(self, rot_speed: float) -> None:
        self._desired_speeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            0, 0, rot_speed, self._pigeon.getRotation2d()
        )

    def get_bot_pose(self) -> Pose3d:
        return Pose3d(self._pose_estimator.getEstimatedPosition())

    def get_robot_pose_2d(self) -> Pose2d:
        return self._pose_estimator.getEstimatedPosition()

    def get_robot_relative_speeds(self) -> ChassisSpeeds:
        states = self._module_states()
        return self._kinematics.toChassisSpeeds(states)

    def set_pose(self, pose: Pose2d) -> None:
        self._pose_estimator.resetPosition(
            self._pigeon.getRotation2d(),
            self._module_positions(),
            pose,
        )

    def set_brake_mode(self, brake: bool) -> None:
        from phoenix6.configs import MotorOutputConfigs
        from phoenix6.signals import NeutralModeValue
        mode = NeutralModeValue.BRAKE if brake else NeutralModeValue.COAST
        for module in self._modules:
            cfg = MotorOutputConfigs().with_neutral_mode(mode)
            module._drive_motor.configurator.apply(cfg)
            module._steer_motor.configurator.apply(cfg)

    def get_auto_command(self):
        if self._auto_chooser is not None:
            return self._auto_chooser.getSelected()
        return None

    # ── @feedback properties ───────────────────────────────────────────────────

    @magicbot.feedback
    def robot_x(self) -> float:
        return self._pose_estimator.getEstimatedPosition().X()

    @magicbot.feedback
    def robot_y(self) -> float:
        return self._pose_estimator.getEstimatedPosition().Y()

    @magicbot.feedback
    def robot_heading_degrees(self) -> float:
        return self._pose_estimator.getEstimatedPosition().rotation().degrees()

    @magicbot.feedback
    def pathplanner_target_x(self) -> float:
        return self._pp_target_pose.X()

    @magicbot.feedback
    def pathplanner_target_y(self) -> float:
        return self._pp_target_pose.Y()

    # ── MagicBot lifecycle ─────────────────────────────────────────────────────

    def execute(self) -> None:
        # Apply desired speeds through kinematics
        module_states = self._kinematics.toSwerveModuleStates(self._desired_speeds)

        # Desaturate wheel speeds to enforce linear velocity limit
        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            module_states,
            constants.RobotChassisLimits.MAX_ROBOT_LINEAR_VELOCITY,
        )

        # Send states to modules
        for module, state in zip(self._modules, module_states):
            module.set_desired_state(state)

        # Reset desired speeds so the robot stops if nothing sets them this loop
        self._desired_speeds = ChassisSpeeds()

        # Update pose estimator with gyro + module positions
        self._pose_estimator.update(
            self._pigeon.getRotation2d(),
            self._module_positions(),
        )

        heading_deg = self.get_robot_pose_2d().rotation().degrees()

        # Update Limelight orientation so MegaTag2 knows where the robot is facing
        LimelightHelpers.SetRobotOrientation(
            constants.CameraConstants.FRONT_CAMERA_NAME, heading_deg, 0, 0, 0, 0, 0
        )
        LimelightHelpers.SetRobotOrientation(
            constants.CameraConstants.BACK_CAMERA_NAME, heading_deg, 0, 0, 0, 0, 0
        )

        # IMU mode: 1 when disabled (external), 4 when enabled (fused)
        imu_mode = 1 if RobotState.isDisabled() else 4
        LimelightHelpers.SetIMUMode(constants.CameraConstants.FRONT_CAMERA_NAME, imu_mode)
        LimelightHelpers.SetIMUMode(constants.CameraConstants.BACK_CAMERA_NAME, imu_mode)

        # MegaTag2 vision fusion
        front_estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
            constants.CameraConstants.FRONT_CAMERA_NAME
        )
        back_estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
            constants.CameraConstants.BACK_CAMERA_NAME
        )

        if front_estimate is not None and front_estimate.tagCount >= 2:
            self._pose_estimator.addVisionMeasurement(
                front_estimate.pose,
                front_estimate.timestampSeconds,
            )

        if back_estimate is not None and back_estimate.tagCount >= 2:
            self._pose_estimator.addVisionMeasurement(
                back_estimate.pose,
                back_estimate.timestampSeconds,
            )

        # Publish per-module telemetry
        for module in self._modules:
            module.publish_telemetry()

    # ── Helpers ────────────────────────────────────────────────────────────────

    def _module_positions(self):
        fl, fr, bl, br = self._modules
        return (fl.get_position(), fr.get_position(), bl.get_position(), br.get_position())

    def _module_states(self):
        fl, fr, bl, br = self._modules
        return (fl.get_state(), fr.get_state(), bl.get_state(), br.get_state())

    @staticmethod
    def _is_red_alliance() -> bool:
        alliance = DriverStation.getAlliance()
        if alliance:
            return alliance == DriverStation.Alliance.kRed
        return False
