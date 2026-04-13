// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.fairportrobotics.frc.posty.TestableSubsystem;
import org.fairportrobotics.frc.posty.test.PostTest;
import org.littletonrobotics.junction.Logger;

import static org.fairportrobotics.frc.posty.assertions.Assertions.*;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
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
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class TurretSubsystem extends TestableSubsystem {
  public enum TurretState {
    INIT,
    HOMING,
    READY;
  }

  private final boolean LOGGING = true;
  private TurretState turretState;
  private DigitalInput turretLimitSwitch;
  private TalonFX turretMotor;
  private PositionVoltage turretControlMode = new PositionVoltage(0);
  private TalonFXSimState turretSim;
  private StatusSignal<Angle> turretMotorPosition;
  private StatusSignal<ForwardLimitValue> turretMotorForwardLimit;
  private StatusSignal<ReverseLimitValue> turretMotorReverseLimit;
  private SparkMax launcherMotor;
  private SparkClosedLoopController launcherControler;

  private double turretOffsetInRotations = 0;

  private Servo hood;

  public TurretSubsystem() {

    // Turret configuration
    turretLimitSwitch = new DigitalInput(Constants.ShooterConstants.TURRET_LIMIT_CHANNEL);
    turretMotor = new TalonFX(Constants.ShooterConstants.TURRET_MOTOR_ID);
    turretMotor.getConfigurator().apply(
      new TalonFXConfiguration()
        .withSlot0(
          new Slot0Configs()
            .withKP(6)
            .withKI(0)
            .withKD(0.3)
            .withKS(0)
        )
        .withMotorOutput(
          new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
        )
        .withCurrentLimits(
          new CurrentLimitsConfigs()
            .withStatorCurrentLimit(50)
        )
        .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(false).withReverseSoftLimitEnable(false))
        .withFeedback(
          new FeedbackConfigs()
            .withFeedbackRotorOffset(0)
        )
    );
    turretMotorPosition = turretMotor.getPosition();
    turretMotorForwardLimit = turretMotor.getForwardLimit();
    turretMotorReverseLimit = turretMotor.getReverseLimit();

    turretSim = turretMotor.getSimState();

    // Launcher configuration
    launcherMotor = new SparkMax(Constants.ShooterConstants.LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig().apply((SparkMaxConfig) SparkMaxConfig.Presets.REV_NEO);
    config.inverted(Constants.ShooterConstants.LAUNCHER_MOTOR_INVERTED);
    config.voltageCompensation(10);
    config.closedLoop.p(0.0002).i(0.000001).d(0.0005); // I = 0.0000001
    config.closedLoop.feedForward.kS(0.2).kV(0.000).kA(0.000);
    // config.closedLoop.allowedClosedLoopError(10, ClosedLoopSlot.kSlot0);
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    config.closedLoop.positionWrappingEnabled(false);
    // config.closedLoopRampRate(0.5);
    // config.smartCurrentLimit(5, 50);
    launcherMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    launcherControler = launcherMotor.getClosedLoopController();

    // Hood configuration
    hood = new Servo(Constants.ShooterConstants.HOOD_SERVO_CHANNEL);
    turretState = TurretState.INIT;
  }

  public void setLauncher(double speed) {
    if(speed == 0)
    {
        launcherMotor.stopMotor();
        return;
    }
    launcherControler.setSetpoint(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  public boolean isLauncherUpToSpeed() {
    // Valid if velcity is within 10% of setpoint.
    // May need to lower this
    return Math.abs(launcherControler.getSetpoint() - launcherMotor.getEncoder().getVelocity()) >= launcherControler.getSetpoint() * 0.03;
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

  public void homeTurret()
  {
    if(turretState != TurretState.INIT){
      return; // Early return, we don't want to home if we already have
    }
    turretMotor.set(Constants.ShooterConstants.HOMING_SPEED);
    turretState = TurretState.HOMING;
  }

  public boolean isTurretReady(){
    return turretState == TurretState.READY;
  }

  public void turretRotateDir(double speed)
  {
    turretMotor.set(speed);
  }

  public double getTurretAngleRobotRelative()
  {
    return getTurretMotorPosition() / Constants.ShooterConstants.TURRET_GEAR_RATIO;
  }

  private double getTurretMotorPosition(){
    return turretMotorPosition.refresh().getValueAsDouble() - turretOffsetInRotations;
  }

  @Override
  public void periodic() {

    if(turretState == TurretState.HOMING)
    {
      turretMotor.set(0.05);

      if(turretLimitSwitch.get())
      { // Turret has tripped the switch
        turretMotor.stopMotor();
        turretOffsetInRotations = (turretMotorPosition.refresh().getValueAsDouble() - 1.323);
        turretState = TurretState.READY;
        // Angle forwardLimit = Angle.ofRelativeUnits(-90, Units.Degrees).times(Constants.ShooterConstants.TURRET_GEAR_RATIO).plus(turretOffset);
        // Angle reverseLimit = Angle.ofRelativeUnits(90, Units.Degrees).times(Constants.ShooterConstants.TURRET_GEAR_RATIO).minus(turretOffset);
        // Logger.recordOutput("TurretSubsystem-TurretForwardLimit", forwardLimit);
        // Logger.recordOutput("TurretSubsystem-TurretReverseLimit", reverseLimit);
        turretMotor.getConfigurator().apply(
          new SoftwareLimitSwitchConfigs()
            // .withForwardSoftLimitEnable(true)
            // .withForwardSoftLimitThreshold(forwardLimit)
            // .withReverseSoftLimitEnable(true)
            // .withReverseSoftLimitThreshold(reverseLimit)
        );
        // setTurretRobotRelative(Angle.ofRelativeUnits(0, Units.Degrees));
        setTurretMotorRotation(-0.34); // Return to 0 after MIAMI VALLEY
      }
    }

    if(LOGGING)
    {
      Logger.recordOutput("TurretSubsystem-TurretLimitSwitch", turretLimitSwitch.get());
      Logger.recordOutput("TurretSubsystem-Launcher Speed (RPM)", launcherMotor.getEncoder().getVelocity());
      Logger.recordOutput("TurretSubsystem-TurretState", turretState);
      Logger.recordOutput("TurretSubsystem-TurretPosition", getTurretAngleRobotRelative());
      Logger.recordOutput("TurretSubsystem-TurretMotorPositionWithOffset", getTurretMotorPosition());
      Logger.recordOutput("TurretSubsystem-TurretMotorPositionRaw", turretMotorPosition.refresh().getValue().in(Units.Degrees));
      Logger.recordOutput("TurretSubsystem-TurretRequestedPosRaw", turretControlMode.getPositionMeasure().in(Units.Degrees));
      Logger.recordOutput("TurretSubsystem-TurretRequestedPos", getRequestedPosTurretRelative());
      Logger.recordOutput("TurretSubsystem-TurretOffset", turretOffsetInRotations);
      Logger.recordOutput("TurretSubsystem-TurretForwardLimitHit", turretMotorForwardLimit.refresh().getValue());
      Logger.recordOutput("TurretSubsystem-TurretReverseLimitHit", turretMotorReverseLimit.refresh().getValue());
    }

  }

  public void setTurretFieldRelative(double pos)
  {
    if(turretState != TurretState.READY)
    {
      // Not ready to set angle
      return;
    }
  }

  public void setTurretRobotRelative(double pos)
  {
    if(turretState != TurretState.READY)
    {
      // Not ready to set angle
      return;
    }

    setTurretMotorRotation(pos * Constants.ShooterConstants.TURRET_GEAR_RATIO);
  }

  public void setTurretMotorRotation(double pos){
    turretMotor.setControl(turretControlMode.withPosition((pos + turretOffsetInRotations)));
    turretSim.setRawRotorPosition(pos + turretOffsetInRotations);
  }

  private double getRequestedPosTurretRelative() {
    return (turretControlMode.getPositionMeasure().in(Units.Rotations) - turretOffsetInRotations) / Constants.ShooterConstants.TURRET_GEAR_RATIO;
  }

  @PostTest()
  public void TurretSubsystem_CANDevicesConnected(){
    assertThat(launcherMotor.getFaults().can).as("Launcher motor not connected!").isFalse();
    assertThat(turretMotor.isConnected()).as("Turret motor not connected!").isTrue();
  }

}
