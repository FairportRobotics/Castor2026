// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.fairportrobotics.frc.posty.TestableSubsystem;
import org.fairportrobotics.frc.posty.test.PostTest;
import org.littletonrobotics.junction.Logger;

import static org.fairportrobotics.frc.posty.assertions.Assertions.*;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class TurretSubsystem extends TestableSubsystem {
  public enum TurretState {
    INIT,
    HOMING,
    MANUAL;
  }

  private final boolean LOGGING = true;
  private TurretState currentState;
  private DigitalInput turretLimitSwitch;
  private TalonFX turretMotor;
  private SparkMax launcherMotor;
  private SparkClosedLoopController launcherControler;

  private Servo hood;
  private Angle limitPos;
  private Angle limitNeg;
  private boolean turretGoingPos;

  public TurretSubsystem() {

    // Turret configuration
    turretLimitSwitch = new DigitalInput(Constants.ShooterConstants.TURRET_LIMIT_CHANNEL);
    turretMotor = new TalonFX(Constants.ShooterConstants.TURRET_MOTOR_ID);
    turretMotor.getConfigurator().apply(
      new TalonFXConfiguration()
        .withSlot0(
          new Slot0Configs()
            .withKP(1.0)
            .withKI(0)
            .withKD(0)
        )
        .withMotorOutput(
          new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
        )
        .withCurrentLimits(
          new CurrentLimitsConfigs()
        )
    );

    // Launcher configuration
    launcherMotor = new SparkMax(Constants.ShooterConstants.LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig().apply((SparkMaxConfig) SparkMaxConfig.Presets.REV_NEO);
    config.inverted(Constants.ShooterConstants.LAUNCHER_MOTOR_INVERTED);
    config.voltageCompensation(10);
    config.closedLoop.p(0.0001).i(0).d(0.0); // I = 0.0000001
    config.closedLoop.feedForward.kS(0.2).kV(0.002).kA(0.00);
    // config.closedLoop.allowedClosedLoopError(10, ClosedLoopSlot.kSlot0);
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    config.closedLoop.positionWrappingEnabled(false);
    // config.closedLoopRampRate(0.5);
    // config.smartCurrentLimit(5, 50);
    launcherMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    launcherControler = launcherMotor.getClosedLoopController();

    // Hood configuration
    hood = new Servo(Constants.ShooterConstants.HOOD_SERVO_CHANNEL);
    currentState = TurretState.INIT;
    limitNeg = Constants.ShooterConstants.LIMIT_AXIMUTH_NEG;
    limitPos = Constants.ShooterConstants.LIMIT_AXIMUTH_POS;
    turretGoingPos = false;
  }

  public void setLauncher(double speed) {
    launcherControler.setSetpoint(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    // launcherMotor.set(speed/1000);
  }

  public boolean isLauncherUpToSpeed() {
    return true;
  }

  public void turretControl(double position)
  {
      turretMotor.setControl(new PositionVoltage(position));
  }

  public void setTargetElevation(double elev)
  {
    double pos = Math.abs((elev - 300)/300);
    Logger.recordOutput("ShooterHood-Angle", pos);
    hood.set(pos);
  }

  public Command revSpeedCommand()
  {
    return this.runOnce(() -> setLauncher(-.5));
  }

  public Command setShooter(int speedRPM){
    return Commands.runEnd(() -> {
      setLauncher(speedRPM);
    }, () -> {
      launcherMotor.stopMotor();
    }, this);
  }

  public void startHoming()
  {
    turretMotor.set(Constants.ShooterConstants.HOMING_SPEED);
    currentState = TurretState.HOMING;
  }

  public void startRotate(double speed)
  {
    turretGoingPos=speed>0;

    if(!passedLimit())
    {
      turretMotor.set(speed);
    }
    else
    {
      turretMotor.set(0);
    }
  }

  public boolean passedLimit(){
    return (getTurretAngle().gt(limitPos) && turretGoingPos) || (getTurretAngle().lt(limitNeg) && !turretGoingPos);
  }

  public Angle getTurretAngle()
  {
    return turretMotor.getPosition().refresh().getValue().times(Constants.ShooterConstants.TURRET_GEAR_RATIO);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (currentState) {
      case HOMING:
        periodicHoming();
        break;
      case MANUAL:
        periodicManual();
        break;

      default:
        break;
    }

    if(LOGGING)
    {
      Logger.recordOutput("TurretSubsystem-Launcher Speed (RPM)", launcherMotor.getEncoder().getVelocity());
      Logger.recordOutput("TurretSubsystem-Launcher Volatge", launcherMotor.getOutputCurrent());
    }

  }

  private void periodicHoming()
  {
    if(turretLimitSwitch.get())
    {
      currentState = TurretState.MANUAL;
      turretMotor.set(0);
      turretMotor.setPosition(0);
      goTo(limitPos);
    }
  }

  private void goTo(Angle pos)
  {
    if(pos.gt(limitNeg) && pos.lt(limitPos))
    {
      while(getTurretAngle()!=pos)
    {
      if(getTurretAngle().gt(pos))
      {
        turretMotor.set(.5);
      }
      else
      {
        turretMotor.set(-.5);
      }
    }
    }
  }

  private void periodicManual()
  {   
    if(passedLimit())
    {
      turretMotor.set(0);
    }
  }

  @PostTest()
  public void TurretSubsystem_CANDevicesConnected(){
    assertThat(launcherMotor.getDeviceId()).as("Launcher motor not connected!").isGreaterThan(0);
    assertThat(turretMotor.isConnected()).as("Turret motor not connected!").isTrue();
  }

}
