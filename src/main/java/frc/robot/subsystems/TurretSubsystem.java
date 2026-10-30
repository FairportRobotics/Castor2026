// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.fairportrobotics.frc.posty.TestableSubsystem;
import org.littletonrobotics.junction.Logger;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class TurretSubsystem extends TestableSubsystem 
{
  private SparkMax launcherMotor;
  private SparkClosedLoopController launcherControler;
  private Servo hood;

  public TurretSubsystem() 
  {
    // Launcher configuration
    launcherMotor = new SparkMax(Constants.ShooterConstants.LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig().apply((SparkMaxConfig) SparkMaxConfig.Presets.REV_NEO);
    config.inverted(Constants.ShooterConstants.LAUNCHER_MOTOR_INVERTED);
    config.voltageCompensation(10);
    config.closedLoop.p(0.0002).i(0.000001).d(0.0005); // I = 0.0000001
    config.closedLoop.feedForward.kS(0.2).kV(0.00005).kA(0.001);
    // config.closedLoop.allowedClosedLoopError(10, ClosedLoopSlot.kSlot0);
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    config.closedLoop.positionWrappingEnabled(false);
    // config.closedLoopRampRate(0.5);
    // config.smartCurrentLimit(5, 50);
    launcherMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    launcherControler = launcherMotor.getClosedLoopController();

    // Hood configuration
    hood = new Servo(Constants.ShooterConstants.HOOD_SERVO_CHANNEL);
  }

  public void setLauncher(double speed) {
    if(speed == 0)
    {
        launcherMotor.stopMotor();
        return;
    }
    launcherControler.setSetpoint(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  public void setTargetElevation(double elev)
  {
    double pos = Math.abs((elev - 300)/300);
    Logger.recordOutput("ShooterHood-Angle", pos);
    hood.set(pos);
  }

  public Command setShooter(int speedRPM){
    return Commands.runEnd(() -> {
      setLauncher(speedRPM);
    }, () -> {
      launcherMotor.stopMotor();
    }, this);
  }
}
