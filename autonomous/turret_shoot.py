"""
autonomous/turret_shoot.py

commands2.Command that replicates AutoTurretShootCommand.java.

Uses distance-based ballistics (Utils.get_hood_angle/launcher_rpm_for_distance)
to set the turret launcher speed and hood angle, then fires once ready.
"""

import commands2
from wpilib import DriverStation

import constants
import utils
from components.drive import DriveComponent
from components.hopper import HopperComponent
from components.intake import IntakeComponent
from components.turret import TurretComponent


class TurretShootCommand(commands2.Command):
    """
    Distance-based auto-shoot command.
    Registered with PathPlanner as "TurretShoot".
    """

    _WAIT_SECONDS: float = 1.5

    def __init__(
        self,
        hopper: HopperComponent,
        turret: TurretComponent,
        intake: IntakeComponent,
        drive: DriveComponent,
    ) -> None:
        super().__init__()
        self._hopper = hopper
        self._turret = turret
        self._intake = intake
        self._drive = drive

        self._wait_timer = commands2.WaitCommand(self._WAIT_SECONDS)
        self._wait_started = False

    def initialize(self) -> None:
        self._hopper.spindexer_on()
        self._wait_timer = commands2.WaitCommand(self._WAIT_SECONDS)
        self._wait_timer.schedule()
        self._wait_started = True

    def execute(self) -> None:
        turret_target = self._turret.get_turret_target_pose()
        bot_pose = self._drive.get_bot_pose()

        if turret_target is not None:
            target_2d = turret_target.toPose2d().translation()
            bot_2d = bot_pose.translation().toTranslation2d()
            distance = target_2d.distance(bot_2d)

            self._turret.set_target_elevation(utils.get_hood_angle_for_distance(distance))
            self._turret.set_launcher(utils.get_launcher_rpm_for_distance(distance))

        if self._turret.is_launcher_up_to_speed() and self._wait_timer.isFinished():
            self._hopper.feed_kicker()
        else:
            self._hopper.stop_kicker()

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        if self._wait_started:
            self._wait_timer.cancel()

        self._hopper.spindexer_off()
        self._hopper.stop_kicker()
        self._turret.set_launcher(0)
        self._turret.set_target_elevation(constants.ShooterConstants.DEFLECTOR_STORED_ANGLE)

        # Restore turret target to hub for the current alliance
        alliance = DriverStation.getAlliance()
        if alliance == DriverStation.Alliance.kBlue:
            self._turret.set_turret_target_pose(constants.FieldPoses.BLUE_HUB_POSE)
        elif alliance == DriverStation.Alliance.kRed:
            self._turret.set_turret_target_pose(constants.FieldPoses.RED_HUB_POSE)
