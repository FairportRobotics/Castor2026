// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class Reverse extends Command {

    private IntakeSubsystem intakeSubsystem;
    private HopperSubsystem hopperSubsystem;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public Reverse(IntakeSubsystem intakeSubsystem, HopperSubsystem hopperSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.hopperSubsystem = hopperSubsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intakeSubsystem.reverseIntake();
        hopperSubsystem.spindexerReverse();
        hopperSubsystem.reverseKicker();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        hopperSubsystem.stopKicker();
        hopperSubsystem.spindexerOff();
        intakeSubsystem.stopIntake();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
