// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.fairportrobotics.frc.posty.TestableSubsystem;
import org.fairportrobotics.frc.posty.test.PostTest;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import static org.fairportrobotics.frc.posty.assertions.Assertions.*;
import frc.robot.Constants;

public class HopperSubsystem extends TestableSubsystem {

    private TalonFX spindexerMotor;
    private TalonFX kickerMotor;
  /** Creates a new HopperSubsystem. */
  public HopperSubsystem() {
    spindexerMotor = new TalonFX(Constants.HopperConstants.SPINDEXER_MOTOR_ID);
    spindexerMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(Constants.HopperConstants.SPINDEXER_MOTOR_DIRECTION));

    kickerMotor = new TalonFX(Constants.HopperConstants.KICKER_MOTOR_ID);
    kickerMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(Constants.HopperConstants.KICKER_MOTOR_DIRECTION));
  } 

  public void feedKicker() {kickerMotor.set(0.75);}

  public void reverseKicker() {kickerMotor.set(-0.5);} 

  public void stopKicker() {kickerMotor.stopMotor();}

  public void spindexerOn() { spindexerMotor.set(0.75);}

  public void spindexerOff() { spindexerMotor.stopMotor();}

  @PostTest(enabled = true)
  public void hopperSystem_CANDevicesConnected(){
    assertThat(kickerMotor.isConnected()).as("Kicker motor not connected!").isTrue();
    assertThat(spindexerMotor.isConnected()).as("Spindexer motor not connected!").isTrue();
  }

}