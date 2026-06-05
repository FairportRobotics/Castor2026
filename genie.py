"""
genie.py – Thin MagicRobot subclass that integrates commands2.CommandScheduler.

GenieRobot bridges MagicBot's component-injection lifecycle with the
commands2/PathPlannerLib command-based pattern by running the
CommandScheduler once per robot loop iteration.
"""

import commands2
import magicbot


class GenieRobot(magicbot.MagicRobot):
    """
    MagicRobot subclass that runs the commands2 CommandScheduler each loop.

    Inherit from this instead of MagicRobot whenever PathPlannerLib commands
    or other commands2 commands need to execute.
    """

    def robotPeriodic(self) -> None:
        """Run the CommandScheduler every loop, then delegate to MagicRobot."""
        commands2.CommandScheduler.getInstance().run()
        return super().robotPeriodic()

    def scheduleCommand(self, command: commands2.Command) -> None:
        """
        Schedule a commands2 Command to run via the CommandScheduler.
        Used to kick off PathPlanner autos and named commands.

        :param command: The command to schedule.
        """
        commands2.CommandScheduler.getInstance().schedule(command)

    def disabledPeriodic(self) -> None:
        """Called periodically while the robot is disabled. No-op by default."""
        pass
