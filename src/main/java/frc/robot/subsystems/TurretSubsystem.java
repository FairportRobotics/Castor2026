// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.fairportrobotics.frc.posty.TestableSubsystem;
import org.fairportrobotics.frc.posty.test.PostTest;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.ConsoleSource.RoboRIO;

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
import frc.robot.Robot;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  private StatusSignal<Double> turretPositionError;
  private double forwardLimit = 0;
  private double reverseLimit = 0;

  private SparkMax launcherMotor;
  private SparkClosedLoopController launcherControler;

  private double turretOffsetInRotations = 0;

  private Servo hood;

  private Pose3d turretTarget;

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
    turretPositionError = turretMotor.getClosedLoopError();

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
  return true;
    // return Math.abs(launcherControler.getSetpoint() - launcherMotor.getEncoder().getVelocity()) >= launcherControler.getSetpoint() * 0.03;
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

  public void setTurretTargetPose(Pose3d targetPose){
    turretTarget = targetPose;
  }

  public Pose3d getTurretTargetPose(){
    return turretTarget;
  }

  private double getTurretMotorPosition(){
    return turretMotorPosition.refresh().getValueAsDouble() - turretOffsetInRotations;
  }

  @Override
  public void periodic() {

    if(turretState == TurretState.HOMING)
    {
      if(turretLimitSwitch.get())
      { // Turret has tripped the switch
        turretMotor.stopMotor();
        turretOffsetInRotations = (turretMotorPosition.refresh().getValueAsDouble() - 1.323);
        if(!Robot.isReal()){
          turretOffsetInRotations = 0; // Flip turret around
        }
        turretState = TurretState.READY;
        forwardLimit = (1.5 + turretOffsetInRotations);
        reverseLimit = (-1.5 + turretOffsetInRotations);
        Logger.recordOutput("TurretSubsystem-TurretForwardLimit", forwardLimit);
        Logger.recordOutput("TurretSubsystem-TurretReverseLimit", reverseLimit);
        turretMotor.getConfigurator().apply(
          new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(forwardLimit)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(reverseLimit)
        );
        // setTurretRobotRelative(0);
        // setTurretMotorRotation(-0.34); // Return to 0 after MIAMI VALLEY
      }
    }

    if(LOGGING)
    {
      Logger.recordOutput("TurretSubsystem-TurretLimitSwitch", turretLimitSwitch.get());
      Logger.recordOutput("TurretSubsystem-Launcher Speed (RPM)", launcherMotor.getEncoder().getVelocity());
      Logger.recordOutput("TurretSubsystem-TurretState", turretState);
      Logger.recordOutput("TurretSubsystem-TurretPositionRobotRelative", getTurretAngleRobotRelative());
      Logger.recordOutput("TurretSubsystem-TurretMotorPositionWithOffset", getTurretMotorPosition());
      Logger.recordOutput("TurretSubsystem-TurretMotorPositionRaw", turretMotorPosition.refresh().getValue().in(Units.Rotations));
      Logger.recordOutput("TurretSubsystem-TurretRequestedPosRaw", turretControlMode.getPositionMeasure().in(Units.Rotations));
      Logger.recordOutput("TurretSubsystem-TurretRequestedPos", getRequestedPosTurretRelative());
      Logger.recordOutput("TurretSubsystem-TurretOffset", turretOffsetInRotations);
      Logger.recordOutput("TurretSubsystem-TurretForwardLimitHit", turretMotorForwardLimit.refresh().getValue());
      Logger.recordOutput("TurretSubsystem-TurretReverseLimitHit", turretMotorReverseLimit.refresh().getValue());
    }

  }

  public void setTurretFieldRelative(Pose3d robotPose, double pos)
  {
    if(turretState != TurretState.READY)
    {
      // Not ready to set angle
      return;
    }

    double correctedPos = -pos; // Flip angle to match CCW positive. Add 0.5 to flip around

    double robotRotations = robotPose.getRotation().toRotation2d().getRotations();

    Rotation2d turretAngle = Rotation2d.fromRotations(correctedPos).plus(Rotation2d.fromRotations(robotRotations)).minus(Rotation2d.fromRotations(0.5));

Logger.recordOutput("TurretSubsystem-TurretFieldTarget", turretAngle.getRotations());

    setTurretRobotRelative(((turretAngle.getRotations())));
  }

  public void setTurretRobotRelative(double pos)
  {
    if(turretState != TurretState.READY)
    {
      // Not ready to set angle
      return;
    }

    double correctedPos = -pos; // Flip angle to match CCW positive

    setTurretMotorRotation(correctedPos * Constants.ShooterConstants.TURRET_GEAR_RATIO);
  }

  public void setTurretMotorRotation(double pos){
    double posWithOffset = pos + turretOffsetInRotations;

    Logger.recordOutput("Blah", posWithOffset);

    if(Robot.isSimulation()){
      if(posWithOffset >= forwardLimit){
        turretSim.setForwardLimit(true);
      } else {
        turretSim.setForwardLimit(false);
      }

      if(posWithOffset <= reverseLimit){
        turretSim.setReverseLimit(true);
      } else {
        turretSim.setReverseLimit(false);
      }
    }

    turretMotor.setControl(turretControlMode.withPosition(posWithOffset));
    turretSim.setRawRotorPosition(-posWithOffset);
  }

  public boolean isTurretAtTarget(){
    return turretPositionError.getValueAsDouble() <= 0.5;
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
