// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.fairportrobotics.frc.posty.TestableSubsystem;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;
import static org.fairportrobotics.frc.robolib.motors.Utils.*;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;

public class TurretSubsystem extends TestableSubsystem {
  public enum TurretState {
    INIT,
    HOMING,
    MANUAL;
  }

  private TurretState currentState;
  private DigitalInput turretLimitSwitch;
  private TalonFX turretMotor;
  private SparkMax launcherMotor;
  private Servo hood;
  private Angle limitPos;
  private Angle limitNeg;
  private boolean turretGoingPos;

  public TurretSubsystem() {
    turretLimitSwitch = new DigitalInput(Constants.ShooterConstants.TURRET_LIMIT_CHANNEL);
    //turretMotor = new TalonFX(Constants.ShooterConstants.TURRET_MOTOR_ID);
    //SetMotorDirection(turretMotor, Constants.ShooterConstants.TURRET_MOTOR_DIRECTION);

    launcherMotor = new SparkMax(Constants.ShooterConstants.LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig()
      .apply((SparkMaxConfig) SparkMaxConfig.Presets.REV_NEO_2);
    config.inverted(Constants.ShooterConstants.LAUNCHER_MOTOR_INVERTED);

    hood = new Servo(Constants.ShooterConstants.HOOD_SERVO_CHANNEL);
    currentState = TurretState.INIT;
    limitNeg = Constants.ShooterConstants.LIMIT_AXIMUTH_NEG;
    limitPos = Constants.ShooterConstants.LIMIT_AXIMUTH_POS;
    turretGoingPos = false;
  }

  public void setLauncher(double speed) {launcherMotor.set(speed);}

  public void turretControl(double position)
  {
      turretMotor.setControl(new PositionVoltage(position));
  }

  public void setTargetElevation(Angle elev)
  {
    if(Constants.ShooterConstants.HOOD_SERVO_INVERTED){
      elev=Units.Degrees.of(300).minus(elev);
    }
    elev=elev.div(1.5);
    elev=elev.div(Constants.ShooterConstants.DEFLECTOR_SERVO_RATIO);
    hood.setAngle(elev.in(Units.Degrees));
  }

  public Command revSpeedCommand()
  {
    return this.runOnce(() -> setLauncher(-.5));
  }

  public void startHoming()
  {
    //turretMotor.set(Constants.ShooterConstants.HOMING_SPEED);
    //currentState = TurretState.HOMING;
  }

  public void startRotate(double speed)
  {
    /*turretGoingPos=speed>0;

    if(!passedLimit())
    {
      turretMotor.set(speed);
    }
    else
    {
      turretMotor.set(0);
    }*/
  }

  public boolean passedLimit(){
    return (getTurretAngle().gt(limitPos) && turretGoingPos) || (getTurretAngle().lt(limitNeg) && !turretGoingPos);
  }

  public Angle getTurretAngle()
  {
    // return turretMotor.getPosition().refresh().getValue().times(Constants.ShooterConstants.TURRET_GEAR_RATIO);
    return Units.Degrees.of(0.0);
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
  }

  private void periodicHoming()
  {
    if(turretLimitSwitch.get())
    {
      currentState = TurretState.MANUAL;
      //turretMotor.set(0);
      //turretMotor.setPosition(0);
    }
  }

  private void periodicManual()
  {
    //If the turret move, it will be a uh-oh, so DONT MOVE IT
    
    /*if(passedLimit())
    {
      //turretMotor.set(0);
    }*/
  }
}
