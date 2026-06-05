"""
components/controller.py – MagicBot-compatible Xbox controller wrapper.

XboxController wraps wpilib.XboxController to provide:
  - Deadband-corrected joystick axes
  - Edge-detection helpers (was_pressed / pressed) for every button
  - Integration with MagicBot's component injection system (``port`` attribute)
"""

import wpilib


class XboxController:
    """
    MagicBot component that wraps a wpilib.XboxController.

    MagicBot injects the ``port`` attribute before ``setup()`` is called,
    so declare it at class level — its value is overridden per-robot in
    ``MyRobot`` via the ``driver_controller_port`` prefixed variable.

    Joystick values are deadband-corrected and cached each loop in
    ``execute()`` so all code within a single loop sees consistent axis values.

    Button helpers follow two patterns:
      - ``*_pressed()``      – True while the button is held down.
      - ``*_was_pressed()``  – True on the loop after the button was released
                               (one-shot, resets immediately after reading).
    """

    #: Whether to apply deadband correction to raw joystick values.
    correct_for_deadband: bool = True
    #: Deadband radius; inputs within ±deadband map to 0.
    deadband: float = 0.1
    #: USB port number; injected by MagicBot from ``driver_controller_port``.
    port: int
    #: Cached left-stick X axis value (deadband-corrected).
    left_x: float = 0.0
    #: Cached left-stick Y axis value (deadband-corrected).
    left_y: float = 0.0
    #: Cached right-stick X axis value (deadband-corrected).
    right_x: float = 0.0
    #: Cached right-stick Y axis value (deadband-corrected).
    right_y: float = 0.0

    def setup(self) -> None:
        """
        Setup the Xbox controller.
        This method is called once when the robot is initialized.
        """
        self.this_controller = wpilib.XboxController(self.port)
        controller_buttons = [
            "A",
            "B",
            "X",
            "Y",
            "DPad_Up",
            "DPad_Down",
            "DPad_Left",
            "DPad_Right",
            "Left_Bumper",
            "Right_Bumper",
            "Left_Trigger",
            "Right_Trigger",
            "Start",
            "Back",
        ]
        self.button_is_pressed = {button: False for button in controller_buttons}
        self.button_was_pressed = {button: False for button in controller_buttons}

    def _button_pressed(self, button_name: str, pressed: bool) -> bool:
        """
        Generic method to check if a button is pressed.
        :param button_name: The name of the button to check.
        :param pressed: The current state of the button (True if pressed, False otherwise).
        """
        if self.button_is_pressed[button_name] and not pressed:
            self.button_was_pressed[button_name] = True
        self.button_is_pressed[button_name] = pressed
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
        if not self.correct_for_deadband:
            return raw_value
        if abs(raw_value) < self.deadband:
            return 0.0
        return (raw_value / abs(raw_value)) * (
            (abs(raw_value) - self.deadband) / (1 - self.deadband)
        )

    def execute(self) -> None:
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
        self.left_y = self._corrected_joystick_value(self.this_controller.getLeftY())
        self.left_x = self._corrected_joystick_value(self.this_controller.getLeftX())
        self.right_y = self._corrected_joystick_value(self.this_controller.getRightY())
        self.right_x = self._corrected_joystick_value(self.this_controller.getRightX())

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
