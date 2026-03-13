// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ManualHopperCommand extends Command {
  private final HopperSubsystem m_subsystem;
  private final XboxController m_xboxController;

  public ManualHopperCommand(HopperSubsystem subsystem, XboxController xboxController) {
    m_subsystem = subsystem;
    m_xboxController = xboxController;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.spin(Constants.HopperConstants.SPINDEXER_MOTOR_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_xboxController.getLeftTriggerAxis()>.5 && m_xboxController.getRightTriggerAxis()>.5)
    {
      m_subsystem.feed();
    }
    else
    {
      m_subsystem.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
