"""
autonomous/chassis_align_shoot.py

commands2.Command that replicates AutoShootCommandChassisTurret.java.

Uses a PID controller to camera-centre the robot on the HUB AprilTags and
dead-reckoning as a fallback.  Fires the kicker when centred and launcher is
up to speed.
"""

import math

import commands2
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from wpilib import DriverStation
from wpimath.controller import PIDController
from wpimath.geometry import Pose3d

import constants
import LimelightHelpers
import utils
from components.drive import DriveComponent
from components.hopper import HopperComponent
from components.turret import TurretComponent


class ChassisAlignShootCommand(commands2.Command):
    """
    Aligns the robot chassis and fires the launcher using camera feedback.
    Registered with PathPlanner as "ChassisAlignShoot".
    """

    _field_tags = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltWelded)

    def __init__(
        self,
        drive: DriveComponent,
        hopper: HopperComponent,
        turret: TurretComponent,
    ) -> None:
        super().__init__()
        self._drive = drive
        self._hopper = hopper
        self._turret = turret

        self._tag_filters: list[int] = []
        self._closest_tag: Pose3d | None = None

        # Camera centering PID (units: degrees)
        self._camera_controller = PIDController(0.15, 0.0, 0.0)
        self._camera_controller.setTolerance(0.2)
        self._camera_controller.setSetpoint(0)

        # Dead-reckoning fallback PID (units: radians)
        self._dead_reckoning_controller = PIDController(math.pi * 0.1, 0, 0.0)
        self._dead_reckoning_controller.setTolerance(0.2)
        self._dead_reckoning_controller.enableContinuousInput(-math.pi, math.pi)

        self._chassis_rotate_speed = math.pi * 0.1

    def initialize(self) -> None:
        alliance = DriverStation.getAlliance()
        if alliance == DriverStation.Alliance.kBlue:
            self._tag_filters = constants.CameraConstants.IDFilters.BLUE_HUB_SHOOTING_IDS
        else:
            self._tag_filters = constants.CameraConstants.IDFilters.RED_HUB_SHOOTING_IDS

        self._turret.set_launcher(3000)
        self._turret.set_target_elevation(constants.ShooterConstants.DEFLECTOR_STORED_ANGLE)
        self._hopper.spindexer_on()
        # Warm-up the camera PID
        self._camera_controller.calculate(-100)

        # Switch to HUB-tracking pipeline
        LimelightHelpers.setPipelineIndex(
            constants.CameraConstants.BACK_CAMERA_NAME,
            constants.CameraConstants.BACK_HUB_TRACKING_3D_PIPELINE_NUMBER,
        )
        LimelightHelpers.SetFiducialIDFiltersOverride(
            constants.CameraConstants.BACK_CAMERA_NAME, self._tag_filters
        )

        # Find the closest HUB AprilTag for dead-reckoning
        bot_pose = self._drive.get_bot_pose()
        tag_poses: list[Pose3d] = [
            pose
            for tag_id in self._tag_filters
            if (pose := self._field_tags.getTagPose(tag_id)) is not None
        ]
        if tag_poses:
            # Find nearest tag by distance
            def _dist(tag: Pose3d) -> float:
                delta = bot_pose.translation().toTranslation2d() - tag.translation().toTranslation2d()
                return math.hypot(delta.X(), delta.Y())
            self._closest_tag = min(tag_poses, key=_dist)

            delta = (
                bot_pose.translation().toTranslation2d()
                - self._closest_tag.translation().toTranslation2d()
            )
            self._dead_reckoning_controller.setSetpoint(
                math.atan2(delta.Y(), delta.X())
            )

    def execute(self) -> None:
        has_target = LimelightHelpers.getTV(constants.CameraConstants.BACK_CAMERA_NAME)

        if not has_target:
            # ── Dead-reckoning fallback ──────────────────────────────────
            # (Chassis rotation was commented out in Java source; preserved here)
            # bot_pose = self._drive.get_bot_pose()
            # self._drive.rotate_chassis(
            #     -self._dead_reckoning_controller.calculate(
            #         bot_pose.rotation().toRotation2d().radians()
            #     )
            # )
            pass
        else:
            # ── Camera centering ─────────────────────────────────────────
            targets_pose = LimelightHelpers.getTargetPose_CameraSpace(
                constants.CameraConstants.BACK_CAMERA_NAME
            )
            if targets_pose and len(targets_pose) >= 3:
                distance_z = targets_pose[2]
                tx = targets_pose[0]

                turret_angle = (
                    utils.calculate_turret_angle_from_camera_tag_distance(distance_z)
                    * constants.ShooterConstants.TURRET_GEAR_RATIO
                )
                self._turret.set_turret_motor_rotation(-turret_angle)

                self._drive.rotate_chassis(
                    -self._camera_controller.calculate(tx)
                )

        # Fire when centred and launcher is up to speed
        if (
            self._turret.is_launcher_up_to_speed()
            and self._camera_controller.atSetpoint()
        ):
            self._hopper.feed_kicker()
        else:
            pass  # Waiting to fire

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self._turret.set_launcher(0)
        self._hopper.stop_kicker()
        self._hopper.spindexer_off()
        self._drive.stop_drive()

        LimelightHelpers.setPipelineIndex(
            constants.CameraConstants.BACK_CAMERA_NAME,
            constants.CameraConstants.BACK_LOCALIZATION_PIPELINE_NUMBER,
        )
