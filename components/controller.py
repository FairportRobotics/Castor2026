import wpilib


class PlayStationController:
    correct_for_deadband: bool
    deadband: float
    port: int
    version: int = 4

    def execute(self) -> None:
        pass

    def setup(self) -> None:
        """
        Setup the PlayStation controller.
        This method is called once when the robot is initialized.
        """
        if self.version == 5:
            self.this_controller = wpilib.PS5Controller(self.port)
        else:
            self.this_controller = wpilib.PS4Controller(self.port)

        self.button_was_pressed = {
            "Cross": False,
            "Circle": False,
            "Square": False,
            "Triangle": False,
            "DPad_Up": False,
            "DPad_Down": False,
            "DPad_Left": False,
            "DPad_Right": False,
            "L1": False,
            "R1": False,
            "L2": False,
            "R2": False,
            "Options": False,
            "Share": False,
        }
        self.button_is_pressed = {
            "Cross": False,
            "Circle": False,
            "Square": False,
            "Triangle": False,
            "DPad_Up": False,
            "DPad_Down": False,
            "DPad_Left": False,
            "DPad_Right": False,
            "L1": False,
            "R1": False,
            "L2": False,
            "R2": False,
            "Options": False,
            "Share": False,
        }

    def _button_pressed(self, button_name: str, pressed: bool) -> bool:
        """
        Generic method to check if a button is pressed.
        :param button_name: The name of the button to check.
        :param pressed: The current state of the button (True if pressed, False otherwise).
        """
        if pressed:
            self.button_is_pressed[button_name] = True
        else:
            if self.button_is_pressed[button_name]:
                self.button_was_pressed[button_name] = True
            self.button_is_pressed[button_name] = False
        return pressed

    def _button_was_pressed(self, button_name: str) -> bool:
        """
        Generic method to check if a button was pressed since the last check.
        :param button_name: The name of the button to check.
        :return: True if the button was pressed since the last check, False otherwise.
        """
        if self.button_was_pressed[button_name]:
            self.button_was_pressed[button_name] = False
            return True
        return False

    def _corrected_joystick_value(self, raw_value: float) -> float:
        """
        Apply deadband correction to a joystick value.
        :param raw_value: The raw joystick value.
        :return: The corrected joystick value.
        """
        if self.correct_for_deadband:
            if abs(raw_value) < self.deadband:
                return 0.0
            return (raw_value / abs(raw_value)) * (
                (abs(raw_value) - self.deadband) / (1 - self.deadband)
            )
        return raw_value

    def capture_button_presses(self) -> None:
        """
        Capture the current state of all buttons to track presses.
        This method should be called at the end of each control loop iteration.
        """
        self.cross_button_pressed()
        self.circle_button_pressed()
        self.square_button_pressed()
        self.triangle_button_pressed()
        self.dpad_up_pressed()
        self.dpad_down_pressed()
        self.dpad_left_pressed()
        self.dpad_right_pressed()
        self.l1_button_pressed()
        self.r1_button_pressed()
        self.l2_button_pressed()
        self.r2_button_pressed()
        self.options_button_pressed()
        self.share_button_pressed()

    def get_joysticks(self) -> tuple[float, float, float, float]:
        """
        Get the joystick values from the Xbox controller.

        :return: A tuple containing the left joystick x, left joystick y, right joystick x, and right joystick y values.
        """
        return (
            self._corrected_joystick_value(self.this_controller.getLeftX()),
            self._corrected_joystick_value(self.this_controller.getLeftY()),
            self._corrected_joystick_value(self.this_controller.getRightX()),
            self._corrected_joystick_value(self.this_controller.getRightY()),
        )

    def cross_button_pressed(self) -> bool:
        """
        Check if the Croiss button is pressed.

        :return: True if the Cross button is pressed, False otherwise.
        """
        return self._button_pressed("Cross", self.this_controller.getCrossButton())

    def cross_button_was_pressed(self) -> bool:
        """
        Check if the Cross button was pressed since the last check.

        :return: True if the Cross button was pressed since the last check, False otherwise.
        """
        return self._button_was_pressed("Cross")

    def circle_button_pressed(self) -> bool:
        """
        Check if the Circle button is pressed.

        :return: True if the Circle button is pressed, False otherwise.
        """
        return self._button_pressed("Circle", self.this_controller.getCircleButton())

    def circle_button_was_pressed(self) -> bool:
        """
        Check if the Circle button was pressed since the last check.

        :return: True if the Circle button was pressed since the last check, False otherwise.
        """
        return self._button_was_pressed("Circle")

    def square_button_pressed(self) -> bool:
        """
        Check if the Square button is pressed.

        :return: True if the Square button is pressed, False otherwise.
        """
        return self._button_pressed("Square", self.this_controller.getSquareButton())

    def square_button_was_pressed(self) -> bool:
        """
        Check if the Square button was pressed since the last check.

        :return: True if the Square button was pressed since the last check, False otherwise.
        """
        return self._button_was_pressed("Square")

    def triangle_button_pressed(self) -> bool:
        """
        Check if the Triangle button is pressed.

        :return: True if the Triangle button is pressed, False otherwise.
        """
        return self._button_pressed(
            "Triangle", self.this_controller.getTriangleButton()
        )

    def triangle_button_was_pressed(self) -> bool:
        """
        Check if the Triangle button was pressed since the last check.

        :return: True if the Triangle button was pressed since the last check, False otherwise.
        """
        return self._button_was_pressed("Triangle")

    def dpad_up_pressed(self) -> bool:
        """
        Check if the D-pad is pressed up.

        :return: True if the D-pad is pressed up, False otherwise.
        """
        return self._button_pressed("DPad_Up", self.this_controller.getPOV() == 0)

    def dpad_up_was_pressed(self) -> bool:
        """
        Check if the D-pad was pressed up since the last check.

        :return: True if the the D-pad was pressed up since the last check, False otherwise.
        """
        return self._button_was_pressed("DPad_Up")

    def dpad_down_pressed(self) -> bool:
        """
        Check if the D-pad is pressed down.

        :return: True if the D-pad is pressed down, False otherwise.
        """
        return self._button_pressed("DPad_Down", self.this_controller.getPOV() == 180)

    def dpad_down_was_pressed(self) -> bool:
        """
        Check if the D-pad was pressed down since the last check.

        :return: True if the the D-pad was pressed down since the last check, False otherwise.
        """
        return self._button_was_pressed("DPad_Down")

    def dpad_left_pressed(self) -> bool:
        """
        Check if the D-pad is pressed left.

        :return: True if the D-pad is pressed left, False otherwise.
        """
        return self._button_pressed("DPad_Left", self.this_controller.getPOV() == 270)

    def dpad_left_was_pressed(self) -> bool:
        """
        Check if the D-pad was pressed left since the last check.

        :return: True if the the D-pad was pressed left since the last check, False otherwise.
        """
        return self._button_was_pressed("DPad_Left")

    def dpad_right_pressed(self) -> bool:
        """
        Check if the D-pad is pressed right.

        :return: True if the D-pad is pressed right, False otherwise.
        """
        return self._button_pressed("DPad_Right", self.this_controller.getPOV() == 90)

    def dpad_right_was_pressed(self) -> bool:
        """
        Check if the D-pad was pressed right since the last check.

        :return: True if the the D-pad was pressed right since the last check, False otherwise.
        """
        return self._button_was_pressed("DPad_Right")

    def r1_button_pressed(self) -> bool:
        """
        Check if the R1 button is pressed.

        :return: True if the R1 button is pressed, False otherwise.
        """
        return self._button_pressed("R1", self.this_controller.getR1Button())

    def r1_button_was_pressed(self) -> bool:
        """
        Check if the R1 button was pressed since the last check.

        :return: True if the R1 button was pressed since the last check, False otherwise.
        """
        return self._button_was_pressed("R1")

    def r2_button_pressed(self) -> bool:
        """
        Check if the R2 button is pressed.

        :return: True if the R2 button is pressed, False otherwise.
        """
        return self._button_pressed("R2", self.this_controller.getR2Button())

    def r2_button_was_pressed(self) -> bool:
        """
        Check if the R2 button was pressed since the last check.

        :return: True if the R2 button was pressed since the last check, False otherwise.
        """
        return self._button_was_pressed("R2")

    def l1_button_pressed(self) -> bool:
        """
        Check if the L1 button is pressed.

        :return: True if the L1 button is pressed, False otherwise.
        """
        return self._button_pressed("L1", self.this_controller.getL1Button())

    def l1_button_was_pressed(self) -> bool:
        """
        Check if the L1 button was pressed since the last check.

        :return: True if the L1 button was pressed since the last check, False otherwise.
        """
        return self._button_was_pressed("L1")

    def l2_button_pressed(self) -> bool:
        """
        Check if the L2 button is pressed.

        :return: True if the L2 button is pressed, False otherwise.
        """
        return self._button_pressed("L2", self.this_controller.getL2Button())

    def l2_button_was_pressed(self) -> bool:
        """
        Check if the L2 button was pressed since the last check.

        :return: True if the L2 button was pressed since the last check, False otherwise.
        """
        return self._button_was_pressed("L2")

    def options_button_pressed(self) -> bool:
        """
        Check if the Options button is pressed.

        :return: True if the Options button is pressed, False otherwise.
        """
        return self._button_pressed("Options", self.this_controller.getOptionsButton())

    def options_button_was_pressed(self) -> bool:
        """
        Check if the Options button was pressed since the last check.

        :return: True if the Options button was pressed since the last check, False otherwise.
        """
        return self._button_was_pressed("Options")

    def share_button_pressed(self) -> bool:
        """
        Check if the Share button is pressed.

        :return: True if the Share button is pressed, False otherwise.
        """
        if self.version == 4:
            return self._button_pressed("Share", self.this_controller.getShareButton())
        else:
            return self._button_pressed("Share", False)

    def share_button_was_pressed(self) -> bool:
        """
        Check if the Share button was pressed since the last check.

        :return: True if the Share button was pressed since the last check, False otherwise.
        """
        return self._button_was_pressed("Share")


class XboxController:
    correct_for_deadband: bool
    deadband: float
    port: int

    def execute(self) -> None:
        pass

    def setup(self) -> None:
        """
        Setup the Xbox controller.
        This method is called once when the robot is initialized.
        """
        self.this_controller = wpilib.XboxController(self.port)
        self.button_was_pressed = {
            "A": False,
            "B": False,
            "X": False,
            "Y": False,
            "DPad_Up": False,
            "DPad_Down": False,
            "DPad_Left": False,
            "DPad_Right": False,
            "Left_Bumper": False,
            "Right_Bumper": False,
            "Left_Trigger": False,
            "Right_Trigger": False,
            "Start": False,
            "Back": False,
        }
        self.button_is_pressed = {
            "A": False,
            "B": False,
            "X": False,
            "Y": False,
            "DPad_Up": False,
            "DPad_Down": False,
            "DPad_Left": False,
            "DPad_Right": False,
            "Left_Bumper": False,
            "Right_Bumper": False,
            "Left_Trigger": False,
            "Right_Trigger": False,
            "Start": False,
            "Back": False,
        }

    def _button_pressed(self, button_name: str, pressed: bool) -> bool:
        """
        Generic method to check if a button is pressed.
        :param button_name: The name of the button to check.
        :param pressed: The current state of the button (True if pressed, False otherwise).
        """
        if pressed:
            self.button_is_pressed[button_name] = True
        else:
            if self.button_is_pressed[button_name]:
                self.button_was_pressed[button_name] = True
            self.button_is_pressed[button_name] = False
        return pressed

    def _button_was_pressed(self, button_name: str) -> bool:
        """
        Generic method to check if a button was pressed since the last check.
        :param button_name: The name of the button to check.
        :return: True if the button was pressed since the last check, False otherwise.
        """
        if self.button_was_pressed[button_name]:
            self.button_was_pressed[button_name] = False
            return True
        return False

    def _corrected_joystick_value(self, raw_value: float) -> float:
        """
        Apply deadband correction to a joystick value.
        :param raw_value: The raw joystick value.
        :return: The corrected joystick value.
        """
        if self.correct_for_deadband:
            if abs(raw_value) < self.deadband:
                return 0.0
            return (raw_value / abs(raw_value)) * (
                (abs(raw_value) - self.deadband) / (1 - self.deadband)
            )
        return raw_value

    def capture_button_presses(self) -> None:
        """
        Capture the current state of all buttons to track presses.
        This method should be called at the end of each control loop iteration.
        """
        self.a_button_pressed()
        self.b_button_pressed()
        self.x_button_pressed()
        self.y_button_pressed()
        self.dpad_up_pressed()
        self.dpad_down_pressed()
        self.dpad_left_pressed()
        self.dpad_right_pressed()
        self.left_bumper_pressed()
        self.right_bumper_pressed()
        self.left_trigger_pressed()
        self.right_trigger_pressed()
        self.start_button_pressed()
        self.back_button_pressed()

    def get_joysticks(self) -> tuple[float, float, float, float]:
        """
        Get the joystick values from the Xbox controller.

        :return: A tuple containing the left joystick x, left joystick y, right joystick x, and right joystick y values.
        """
        return (
            self._corrected_joystick_value(self.this_controller.getLeftX()),
            self._corrected_joystick_value(self.this_controller.getLeftY()),
            self._corrected_joystick_value(self.this_controller.getRightX()),
            self._corrected_joystick_value(self.this_controller.getRightY()),
        )

    def a_button_pressed(self) -> bool:
        """
        Check if the A button is pressed.

        :return: True if the A button is pressed, False otherwise.
        """
        return self._button_pressed("A", self.this_controller.getAButton())

    def a_button_was_pressed(self) -> bool:
        """
        Check if the A button was pressed since the last check.

        :return: True if the A button was pressed since the last check, False otherwise.
        """
        return self._button_was_pressed("A")

    def b_button_pressed(self) -> bool:
        """
        Check if the B button is pressed.
        :return: True if the B button is pressed, False otherwise.
        """
        return self._button_pressed("B", self.this_controller.getBButton())

    def b_button_was_pressed(self) -> bool:
        """
        Check if the B button was pressed since the last check.

        :return: True if the B button was pressed since the last check, False otherwise.
        """
        return self._button_was_pressed("B")

    def x_button_pressed(self) -> bool:
        """
        Check if the X button is pressed.

        :return: True if the X button is pressed, False otherwise.
        """
        return self._button_pressed("X", self.this_controller.getXButton())

    def x_button_was_pressed(self) -> bool:
        """
        Check if the X button was pressed since the last check.

        :return: True if the X button was pressed since the last check, False otherwise.
        """
        return self._button_was_pressed("X")

    def y_button_pressed(self) -> bool:
        """
        Check if the Y button is pressed.

        :return: True if the Y button is pressed, False otherwise.
        """
        return self._button_pressed("Y", self.this_controller.getYButton())

    def y_button_was_pressed(self) -> bool:
        """
        Check if the Y button was pressed since the last check.

        :return: True if the Y button was pressed since the last check, False otherwise.
        """
        return self._button_was_pressed("Y")

    def dpad_up_pressed(self) -> bool:
        """
        Check if the D-pad is pressed up.

        :return: True if the D-pad is pressed up, False otherwise.
        """
        return self._button_pressed("DPad_Up", self.this_controller.getPOV() == 0)

    def dpad_up_was_pressed(self) -> bool:
        """
        Check if the D-pad was pressed up since the last check.

        :return: True if the the D-pad was pressed up since the last check, False otherwise.
        """
        return self._button_was_pressed("DPad_Up")

    def dpad_down_pressed(self) -> bool:
        """
        Check if the D-pad is pressed down.

        :return: True if the D-pad is pressed down, False otherwise.
        """
        return self._button_pressed("DPad_Down", self.this_controller.getPOV() == 180)

    def dpad_down_was_pressed(self) -> bool:
        """
        Check if the D-pad was pressed down since the last check.

        :return: True if the the D-pad was pressed down since the last check, False otherwise.
        """
        return self._button_was_pressed("DPad_Down")

    def dpad_left_pressed(self) -> bool:
        """
        Check if the D-pad is pressed left.

        :return: True if the D-pad is pressed left, False otherwise.
        """
        return self._button_pressed("DPad_Left", self.this_controller.getPOV() == 270)

    def dpad_left_was_pressed(self) -> bool:
        """
        Check if the D-pad was pressed left since the last check.

        :return: True if the the D-pad was pressed left since the last check, False otherwise.
        """
        return self._button_was_pressed("DPad_Left")

    def dpad_right_pressed(self) -> bool:
        """
        Check if the D-pad is pressed right.

        :return: True if the D-pad is pressed right, False otherwise.
        """
        return self._button_pressed("DPad_Right", self.this_controller.getPOV() == 90)

    def dpad_right_was_pressed(self) -> bool:
        """
        Check if the D-pad was pressed right since the last check.

        :return: True if the the D-pad was pressed right since the last check, False otherwise.
        """
        return self._button_was_pressed("DPad_Right")

    def left_bumper_pressed(self) -> bool:
        """
        Check if the left bumper is pressed.

        :return: True if the left bumper is pressed, False otherwise.
        """
        return self._button_pressed("Left_Bumper", self.this_controller.getLeftBumper())

    def left_bumper_was_pressed(self) -> bool:
        """
        Check if the left bumper was pressed since the last check.

        :return: True if the left bumper was pressed since the last check, False otherwise.
        """
        return self._button_was_pressed("Left_Bumper")

    def right_bumper_pressed(self) -> bool:
        """
        Check if the right bumper is pressed.

        :return: True if the right bumper is pressed, False otherwise.
        """
        return self._button_pressed(
            "Right_Bumper", self.this_controller.getRightBumper()
        )

    def right_bumper_was_pressed(self) -> bool:
        """
        Check if the right bumper was pressed since the last check.

        :return: True if the right bumper was pressed since the last check, False otherwise.
        """
        return self._button_was_pressed("Right_Bumper")

    def left_trigger_pressed(self) -> float:
        """
        Get the value of the left trigger.

        :return: The value of the left trigger, ranging from 0.0 to 1.0.
        """
        self._button_pressed(
            "Left_Trigger", self.this_controller.getLeftTriggerAxis() >= 0.5
        )
        return self.this_controller.getLeftTriggerAxis()

    def left_trigger_was_pressed(self) -> bool:
        """
        Check if the left trigger was pressed since the last check.

        :return: True if the left trigger was pressed since the last check, False otherwise.
        """
        return self._button_was_pressed("Left_Trigger")

    def right_trigger_pressed(self) -> float:
        """
        Get the value of the right trigger.

        :return: The value of the right trigger, ranging from 0.0 to 1.0.
        """
        self._button_pressed(
            "Right_Trigger", self.this_controller.getRightTriggerAxis() >= 0.5
        )
        return self.this_controller.getRightTriggerAxis()

    def right_trigger_was_pressed(self) -> bool:
        """
        Check if the right trigger was pressed since the last check.

        :return: True if the right trigger was pressed since the last check, False otherwise.
        """
        return self._button_was_pressed("Right_Trigger")

    def start_button_pressed(self) -> bool:
        """
        Check if the start button is pressed.

        :return: True if the start button is pressed, False otherwise.
        """
        return self._button_pressed("Start", self.this_controller.getStartButton())

    def start_button_was_pressed(self) -> bool:
        """
        Check if the start button was pressed since the last check.

        :return: True if the start button was pressed since the last check, False otherwise.
        """
        return self._button_was_pressed("Start")

    def back_button_pressed(self) -> bool:
        """
        Check if the back button is pressed.

        :return: True if the back button is pressed, False otherwise.
        """
        return self._button_pressed("Back", self.this_controller.getBackButton())

    def back_button_was_pressed(self) -> bool:
        """
        Check if the back button was pressed since the last check.

        :return: True if the back button was pressed since the last check, False otherwise.
        """
        return self._button_was_pressed("Back")
