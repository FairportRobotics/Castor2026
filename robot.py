"""
robot.py – Main robot file for Castor 2026 (Python/MagicBot port).

MyRobot extends GenieRobot (which extends MagicRobot and also runs the
CommandScheduler each loop so PathPlannerLib commands work seamlessly).
"""

import commands2
import wpilib
from pathplannerlib.auto import NamedCommands
from pathplannerlib.config import RobotConfig
import constants
from autonomous.chassis_align_shoot import ChassisAlignShootCommand
from autonomous.turret_shoot import TurretShootCommand
from components.controller import XboxController
from components.drive import DriveComponent
from components.hopper import HopperComponent
from components.intake import IntakeComponent
from components.turret import TurretComponent
from genie import GenieRobot


class _DriveSubsystemAdapter(commands2.Subsystem):
    """
    Satisfies AutoBuilder.configure()'s subsystem requirement parameter.
    DriveComponent is not a commands2.Subsystem, so this adapter bridges the gap.
    """

    pass


class MyRobot(GenieRobot):
    # ── MagicBot component declarations (framework auto-injects instances) ──
    driver_controller: XboxController
    # Prefixed variable: MagicBot strips "driver_controller_" and injects
    # `port` into the XboxController component before its setup() runs.
    driver_controller_port: int = constants.OperatorConstants.CONTROLLER_PORT
    drive: DriveComponent
    hopper: HopperComponent
    intake: IntakeComponent
    turret: TurretComponent

    def createObjects(self) -> None:
        """
        Called by MagicBot before any component setup() methods.
        Class-level prefixed variables (e.g. driver_controller_port) are
        injected into components automatically after this method returns.
        """
        # Tracks the active A/X turret-shoot command so it can be cancelled
        self._shoot_cmd: TurretShootCommand | None = None

    def _post_setup(self) -> None:
        """
        Called after all component setup() methods have run (from robotInit).
        Wire button bindings and PathPlanner named commands here because
        component objects are fully constructed at this point.
        """
        # Pass the underlying wpilib controller to intake for rumble feedback
        self.intake.set_controller(self.driver_controller.this_controller)

        # ── PathPlanner named commands ─────────────────────────────────────
        NamedCommands.registerCommand(
            "ChassisAlignShoot",
            ChassisAlignShootCommand(self.drive, self.hopper, self.turret),
        )
        NamedCommands.registerCommand(
            "TurretShoot",
            TurretShootCommand(self.hopper, self.turret, self.intake, self.drive),
        )
        NamedCommands.registerCommand(
            "RunIntake",
            commands2.FunctionalCommand(
                onInit=self.intake.intake,
                onExecute=lambda: None,
                onEnd=lambda interrupted: self.intake.stop_intake(),
                isFinished=lambda: False,
            ),
        )
        # Aliases used in PathPlanner auto files
        NamedCommands.registerCommand(
            "Shoot preload",
            TurretShootCommand(self.hopper, self.turret, self.intake, self.drive),
        )
        NamedCommands.registerCommand(
            "Shoot Headed R-Tr",
            ChassisAlignShootCommand(self.drive, self.hopper, self.turret),
        )
        NamedCommands.registerCommand(
            "Shoot headed L-Tr",
            ChassisAlignShootCommand(self.drive, self.hopper, self.turret),
        )

        # ── AutoBuilder / auto chooser ─────────────────────────────────────
        try:
            robot_config = RobotConfig.fromGUISettings()
            drive_adapter = _DriveSubsystemAdapter()
            self.drive.inject_drive_adapter(drive_adapter, robot_config)
        except Exception as e:
            print(f"[robot.py] AutoBuilder configuration failed: {e}")

    def teleopPeriodic(self) -> None:
        # ── Drive ──────────────────────────────────────────────────────────
        self.drive.set_chassis_speeds_field_relative(
            -self.driver_controller.left_y,
            -self.driver_controller.left_x,
            -self.driver_controller.right_x,
        )

        # ── One-shot actions (fire when button is released) ────────────────
        if self.driver_controller.left_bumper_was_pressed():
            self.turret.home_turret()

        if self.driver_controller.right_bumper_was_pressed():
            self.intake.intake()

        if self.driver_controller.y_button_was_pressed():
            self.scheduleCommand(
                commands2.SequentialCommandGroup(
                    commands2.InstantCommand(self.intake.deploy),
                    commands2.WaitCommand(2.0),
                    commands2.InstantCommand(self.intake.stop_deploy),
                )
            )

        # ── Held actions ───────────────────────────────────────────────────
        if self.driver_controller.start_button_pressed():
            self.intake.reset_deploy()

        if self.driver_controller.b_button_pressed():
            self.intake.reverse_intake()
            self.hopper.reverse_kicker()

        # A or X held → schedule turret-shoot command; cancel when released
        shooting = (
            self.driver_controller.a_button_pressed()
            or self.driver_controller.x_button_pressed()
        )
        if shooting:
            if self._shoot_cmd is None or self._shoot_cmd.isFinished():
                self._shoot_cmd = TurretShootCommand(
                    self.hopper, self.turret, self.intake, self.drive
                )
                self.scheduleCommand(self._shoot_cmd)
        elif self._shoot_cmd is not None and not self._shoot_cmd.isFinished():
            self._shoot_cmd.cancel()
            self._shoot_cmd = None

    def autonomousInit(self) -> None:
        auto_cmd = self.drive.get_auto_command()
        if auto_cmd is not None:
            self.scheduleCommand(auto_cmd)

    def disabledInit(self) -> None:
        self.drive.set_brake_mode(True)

    def robotInit(self) -> None:
        super().robotInit()
        self.drive.set_brake_mode(False)
        self._post_setup()


if __name__ == "__main__":
    wpilib.run(MyRobot)
