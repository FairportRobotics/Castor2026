// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. **/
public class SetDeflectorCommand extends Command {
  private final TurretSubsystem m_subsystem;
  private double deployAngle;

  public SetDeflectorCommand(TurretSubsystem subsystem, double deployAngle) {
    m_subsystem = subsystem;
    this.deployAngle = deployAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setTargetElevation(deployAngle);
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
