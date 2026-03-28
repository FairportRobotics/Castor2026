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
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
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
  private SparkMax launcherMotor;
  private SparkClosedLoopController launcherControler;

  private Angle turretOffset = Angle.ofBaseUnits(0, Units.Degrees);

  private Servo hood;

  public TurretSubsystem() {

    // Turret configuration
    turretLimitSwitch = new DigitalInput(Constants.ShooterConstants.TURRET_LIMIT_CHANNEL);
    turretMotor = new TalonFX(Constants.ShooterConstants.TURRET_MOTOR_ID);
    turretMotor.getConfigurator().apply(
      new TalonFXConfiguration()
        .withSlot0(
          new Slot0Configs()
            .withKP(2)
            .withKI(1)
            .withKD(0)
            .withKS(0)
        )
        .withMotorOutput(
          new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
        )
        .withCurrentLimits(
          new CurrentLimitsConfigs()
            .withStatorCurrentLimit(50)
        )
        .withSoftwareLimitSwitch(
          new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(Angle.ofRelativeUnits(90, Units.Degree).times(Constants.ShooterConstants.TURRET_GEAR_RATIO))
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(Angle.ofRelativeUnits(-90, Units.Degree).times(Constants.ShooterConstants.TURRET_GEAR_RATIO))
        )
        .withFeedback(
          new FeedbackConfigs()
            .withFeedbackRotorOffset(0)
        )
    );

    turretSim = turretMotor.getSimState();

    // Launcher configuration
    launcherMotor = new SparkMax(Constants.ShooterConstants.LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig().apply((SparkMaxConfig) SparkMaxConfig.Presets.REV_NEO);
    config.inverted(Constants.ShooterConstants.LAUNCHER_MOTOR_INVERTED);
    config.closedLoop.p(1).i(0).d(0.1);
    config.closedLoop.allowedClosedLoopError(10, ClosedLoopSlot.kSlot0);
    launcherMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    launcherControler = launcherMotor.getClosedLoopController();

    // Hood configuration
    hood = new Servo(Constants.ShooterConstants.HOOD_SERVO_CHANNEL);
    turretState = TurretState.INIT;
  }

  public void setLauncher(double speed) {launcherControler.setSetpoint(speed, ControlType.kVelocity);}

  public boolean isLauncherUpToSpeed() {
    return launcherControler.isAtSetpoint();
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

  public void homeTurret(){
    turretState = TurretState.HOMING;
  }

  public boolean isTurretReady(){
    return turretState == TurretState.READY;
  }

  public void turretRotateDir(double speed)
  {
    turretMotor.set(speed);
  }

  public Angle getTurretAngle()
  {
    return getTurretMotorPosition().div(Constants.ShooterConstants.TURRET_GEAR_RATIO).minus(Angle.ofRelativeUnits(180, Units.Degrees)).times(-1);
  }

  private Angle getTurretMotorPosition(){
    return turretMotor.getPosition().refresh().getValue().minus(turretOffset);
  }

  @Override
  public void periodic() {

    if(turretState == TurretState.HOMING)
    {
      turretMotor.set(-0.05);

      if(turretLimitSwitch.get())
      { // Turret has tripped the switch
        turretMotor.stopMotor();
        turretOffset = turretMotor.getPosition().refresh().getValue().plus(Angle.ofRelativeUnits(85, Units.Degrees).times(Constants.ShooterConstants.TURRET_GEAR_RATIO));
        turretState = TurretState.READY;
        setTurretRotation(Angle.ofRelativeUnits(0, Units.Degrees));
      }
    }

    if(LOGGING)
    {
      Logger.recordOutput("TurretSubsystem-TurretLimitSwitch", turretLimitSwitch.get());
      Logger.recordOutput("TurretSubsystem-Launcher Speed (RPM)", launcherMotor.getEncoder().getVelocity());
      Logger.recordOutput("TurretSubsystem-TurretState", turretState);
      Logger.recordOutput("TurretSubsystem-TurretPosition", getTurretAngle().in(Units.Degrees));
      Logger.recordOutput("TurretSubsystem-TurretMotorPosition", getTurretMotorPosition().in(Units.Degrees));
      Logger.recordOutput("TurretSubsystem-TurretRequestedPos", turretControlMode.getPositionMeasure().in(Units.Degrees));
    }

  }

  public void setTurretRotation(Angle pos)
  {
    if(turretState != TurretState.READY)
    {
      // Not ready to set angle
      return;
    }

    if(pos.gt(Angle.ofRelativeUnits(90, Units.Degrees)) || pos.lt(Angle.ofRelativeUnits(-90, Units.Degrees)))
    {
      // Don't allow to over extend
      return;
    }

    setTurretMotorRotation(pos.times(Constants.ShooterConstants.TURRET_GEAR_RATIO));
  }

  public void setTurretMotorRotation(Angle pos){
    turretMotor.setControl(turretControlMode.withPosition(pos.plus(turretOffset)));
    turretSim.setRawRotorPosition(pos);
  }

  @PostTest()
  public void TurretSubsystem_CANDevicesConnected(){
    assertThat(launcherMotor.getFaults().can).as("Launcher motor not connected!").isTrue();
    assertThat(turretMotor.isConnected()).as("Turret motor not connected!").isTrue();
  }

}
