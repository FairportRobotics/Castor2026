// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ManualShootCommand extends Command {
  private final TurretSubsystem m_subsystem;
  private final XboxController m_controller;

  public ManualShootCommand(TurretSubsystem subsystem, XboxController controller) {
    m_subsystem = subsystem;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_controller.getLeftTriggerAxis()>.5 || m_controller.getRightTriggerAxis()>.5)
    {
      m_subsystem.setLauncher(1);
    }
    else
    {
      m_subsystem.setLauncher(0);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
