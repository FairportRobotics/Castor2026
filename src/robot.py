import components
import constants
import genie
from pykit.logger import Logger

class MyRobot(genie.GenieRobot):
    controller: components.XboxController
    #pigeon: components.Pigeon
    logger: Logger

    def createObjects(self):
        """Create motors and stuff here"""
        # Controller stuff here
        self.controller_correct_for_deadband = constants.CONTROLLER_CORRECT_FOR_DEADBAND
        self.controller_deadband = constants.CONTROLLER_DEADBAND
        self.controller_port = constants.CONTROLLER_PORT

    def teleopInit(self):
        """Called when teleop starts; optional"""

    def teleopPeriodic(self):
        """Called periodically during teleop"""
        self.controller.capture_button_presses()
