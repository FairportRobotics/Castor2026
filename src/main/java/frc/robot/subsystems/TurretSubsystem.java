// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.fairportrobotics.frc.posty.TestableSubsystem;
import org.fairportrobotics.frc.posty.test.PostTest;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static org.fairportrobotics.frc.posty.assertions.Assertions.*;
import frc.robot.Constants;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
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
  private CANcoder hoodEncoder;
  private Angle limitPos;
  private Angle limitNeg;
  private boolean turretGoingPos;
  private Angle targElev;

  public TurretSubsystem() {
    turretLimitSwitch = new DigitalInput(Constants.ShooterConstants.LIMIT_CHANNEL);
    turretMotor = new TalonFX(Constants.ShooterConstants.TURRET_ID);

    launcherMotor = new SparkMax(Constants.ShooterConstants.LAUNCHER_ID, MotorType.kBrushless);

    hood = new Servo(Constants.ShooterConstants.HOOD_CHANNEL);
    hoodEncoder = new CANcoder(Constants.ShooterConstants.HOOD_ENCODER_ID);
    currentState = TurretState.INIT;
    limitNeg = Constants.ShooterConstants.LIMIT_AXIMUTH_NEG;
    limitPos = Constants.ShooterConstants.LIMIT_AXIMUTH_POS;
    turretGoingPos = false;
    targElev = edu.wpi.first.units.Units.Degrees.of(45);
  }

  public void setLauncher(double speed)
  {
    launcherMotor.set(speed);
  }

  public void setTargetElevation(Angle elev)
  {
    if(elev.gt(Constants.ShooterConstants.LIMIT_ELEVATION_NEG) && elev.lt(Constants.ShooterConstants.LIMIT_ELEVATION_POS))
    {
      targElev=elev;
    }
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
  }

  private void periodicHoming()
  {
    if(turretLimitSwitch.get())
    {
      currentState = TurretState.MANUAL;
      turretMotor.set(0);
      turretMotor.setPosition(0);
    }
  }

  private void periodicManual()
  {
    if(passedLimit())
    {
      turretMotor.set(0);
    }

    Angle curElev = hoodEncoder.getAbsolutePosition().getValue();
    if(targElev.minus(curElev).abs(Units.Degrees)>Constants.ShooterConstants.TARGET_ELEVATION_DIF.in(Units.Degrees))
    {
      if(targElev.gt(curElev))
      {
        hood.set(50);
      }
      else
      {
        hood.set(-50);
      }
    }
    else
    {
      hood.set(0);
    }
  }
}
