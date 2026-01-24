import components
import constants
import genie


class MyRobot(genie.GenieRobot):
    CONTROLLER: components.XboxController

    def createObjects(self):
        """Create motors and stuff here"""
        # Controller stuff here
        self.CONTROLLER_CORRECT_FOR_DEADBAND = constants.CONTROLLER_CORRECT_FOR_DEADBAND
        self.CONTROLLER_DEADBAND = constants.CONTROLLER_DEADBAND
        self.CONTROLLER_PORT = constants.CONTROLLER_PORT

    def teleopInit(self):
        """Called when teleop starts; optional"""

    def teleopPeriodic(self):
        """Called periodically during teleop"""
        self.CONTROLLER.capture_button_presses()
