// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.fairportrobotics.frc.posty.TestableSubsystem;
import org.fairportrobotics.frc.posty.test.PostTest;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import static org.fairportrobotics.frc.posty.assertions.Assertions.*;

import java.time.chrono.ThaiBuddhistChronology;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.wpilibj.DigitalOutput;
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
    deployMotor = new WPI_TalonSRX(Constants.IntakeConstants.DEPLOY_MOTOR_ID);
    deployMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    deployMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
  }

  public void neutral()
  {
    deployMotor.set(0);
  }

  public void extend()
  {
    deployMotor.set(1);
  }

  public void retract()
  {
    deployMotor.set(-1);
  }

  public void setSpeed(double speed)
  {
    intakeMotor.set(speed);
  }

  public boolean isDeployed()
  {
    return deployMotor.isFwdLimitSwitchClosed()==1;
  }

  public boolean isRetracted()
  {
    return deployMotor.isRevLimitSwitchClosed()==1;
  }

  public Command deployCommand()
  {
    return new FunctionalCommand(this::extend, this::extend, interrupted -> neutral(), this::isDeployed, this).andThen(Commands.startEnd(() -> setSpeed(50), () -> setSpeed(0), this));
  }

  public Command reverseCommand()
  {
    return new FunctionalCommand(this::extend, this::extend, interrupted -> neutral(), this::isDeployed, this).andThen(Commands.startEnd(() -> setSpeed(-50), () -> setSpeed(0), this));
  }

  public Command retractCommand()
  {
    return this.runOnce(() -> setSpeed(0)).andThen(new FunctionalCommand(this::retract, this::retract, interrupted -> neutral(), this::isRetracted, this));
  }

  @PostTest(name = "A friendly name", enabled = true)
  public void myFailingPostTest(){
    assertThat(true).isFalse();
  }

  @PostTest(enabled = true)
  public void myPassingPostTest(){
    assertThat("Hello World").endWith("World");
  }
}
