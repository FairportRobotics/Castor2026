// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.fairportrobotics.frc.posty.TestableSubsystem;
import org.fairportrobotics.frc.posty.test.PostTest;

import com.ctre.phoenix6.hardware.TalonFX;

import static org.fairportrobotics.frc.posty.assertions.Assertions.*;
import static org.fairportrobotics.frc.robolib.motors.Utils.*;
import frc.robot.Constants;

public class HopperSubsystem extends TestableSubsystem {

    private TalonFX spindexerMotor;
    private TalonFX kickerMotor;
  /** Creates a new HopperSubsystem. */
  public HopperSubsystem() {
    spindexerMotor = new TalonFX(Constants.HopperConstants.SPINDEXER_MOTOR_ID);
    SetMotorDirection(spindexerMotor, Constants.HopperConstants.SPINDEXER_MOTOR_DIRECTION);

    kickerMotor = new TalonFX(Constants.HopperConstants.KICKER_MOTOR_ID);
    SetMotorDirection(kickerMotor, Constants.HopperConstants.KICKER_MOTOR_DIRECTION);
  } 

  public void feed() {kickerMotor.set(0.5);}

  public void reverse() {kickerMotor.set(-0.5);} 

  public void stop() {kickerMotor.set(0);}

  public void spin(double speed) {spindexerMotor.set(speed);}

  @PostTest(name = "A friendly name", enabled = true)
  public void myFailingPostTest(){
    assertThat(true).isFalse();
  }

  @PostTest(enabled = true)
  public void myPassingPostTest(){
    assertThat("Hello World").endWith("World");
  }
}