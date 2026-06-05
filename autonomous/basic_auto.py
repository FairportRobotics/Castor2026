"""
autonomous/basic_auto.py

Fallback MagicBot AutonomousStateMachine used when PathPlanner is unavailable.
Deploys the intake, homes the turret, then stops.
"""

import magicbot
from magicbot.state_machine import state, timed_state

from components.drive import DriveComponent
from components.intake import IntakeComponent
from components.turret import TurretComponent


class BasicAuto(magicbot.AutonomousStateMachine):
    """Simple fallback autonomous: deploy intake → home turret → idle."""

    MODE_NAME = "Basic Auto"
    DEFAULT = False

    # MagicBot injects these
    drive: DriveComponent
    intake: IntakeComponent
    turret: TurretComponent

    @timed_state(first=True, duration=2.0, next_state="home_turret")
    def deploy_intake(self) -> None:
        self.intake.deploy()

    @timed_state(duration=3.0, next_state="idle")
    def home_turret(self) -> None:
        self.intake.stop_deploy()
        self.turret.home_turret()

    @state
    def idle(self) -> None:
        self.drive.stop_drive()
