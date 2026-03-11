# Top Priority (for manual shooting functionality by RIT)

These are the jobs to be completed first, to ensure we have minimal functionality for RIT

- Add subsystems to robot
    - Define unique IDs and channels for all devices in Constants.java (IP Dom)
    - Add subsystems to the RobotContainer X

- Updates to subsystems X
    - Update launcher wheel to a Neo/SparkMax X

- Define basic commands for manual fire mode
    - Y/Top button: Deploy the intake (and run the rollers) (TO MAP)
        - Could be a single command
        - Could be 2 commands in sequence

    - B/Right button: Reverse the intake (TO MAP)
        - Unclear if this should touch the deploy motor. ATM suggest leaving the deploy motor alone

    - A/Bottom button: Retract the intake (and stop the rollers) (TO MAP)

    - D-pad: Set deflector to pre-programmed angle
        - Suggestion: Implement this once, and make the angle an member variable of the command (TO MAP)

    - Both Triggers: Spin up flywheel (if not already spun up) and then start the kicker.
        - Re: Sensing flywheel speed
            - Could be a time offset (i.e. wait for N seconds for the motor to spin up)
            - Ideally, check the encoder speed (will need the encoders for auto launch)

- Advanced movement functions:
    - L Bumper: "Nudging" mode:
        - On press: 
            - Briefly move the robot forward then stop
            - Trigger nudge at 1 to 2 hz (to be determined, control via constant)
            - Duty cycle (i.e. what percentage of the time the robot is actively moving) is undefined. Control via constant
        - On release:
            - Return to normal driving mode

    - R Bumper: "Trench Maneuver"
        - On button press/enter:
            - Move deflector to lowest position
            - Deploy the intake, no need to start the rollers if not started
        - On button release/exit:
            - Return the deflector to its previous state
            - Restore intake to previous state

# Secondary priority

These are to be completed after the top priority tasks. Currently, they focus on getting automatic targeting working.

- Integrate limelight positioning

- Integrate automatic aiming
