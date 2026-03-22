// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.fairportrobotics.frc.posty.TestableSubsystem;
import org.fairportrobotics.frc.posty.test.PostTest;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

import static org.fairportrobotics.frc.posty.assertions.Assertions.*;
import static org.fairportrobotics.frc.robolib.motors.Utils.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class IntakeSubsystem extends TestableSubsystem {
  /** Creates a new ExampleSubsystem. */

  private TalonFX intakeMotor;
  private WPI_TalonSRX deployMotor;

  public IntakeSubsystem() 
  {
    intakeMotor = new TalonFX(Constants.IntakeConstants.INTAKE_MOTOR_ID);
    SetMotorDirection(intakeMotor, Constants.IntakeConstants.INTAKE_MOTOR_DIRECTION);

    deployMotor = new WPI_TalonSRX(Constants.IntakeConstants.DEPLOY_MOTOR_ID);
    deployMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    deployMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    deployMotor.setInverted(Constants.IntakeConstants.DEPLOY_MOTOR_INVERTED);
  }

  public void extend()
  {
    deployMotor.set(1);
  }

  public void setSpeed(double speed)
  {
    intakeMotor.set(speed);
  }

  public Command startSpeedCommand()
  {
    return this.runOnce(() -> setSpeed(-.5));
  }

  public Command killSpeedCommand()
  {
    return this.runOnce(() -> setSpeed(0));
  }

  public Command revSpeedCommand()
  {
    return this.runOnce(() -> setSpeed(.5));
  }
}
