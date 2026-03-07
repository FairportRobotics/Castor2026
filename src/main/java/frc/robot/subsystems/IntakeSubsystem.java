// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.fairportrobotics.frc.posty.TestableSubsystem;
import org.fairportrobotics.frc.posty.test.PostTest;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import static org.fairportrobotics.frc.posty.assertions.Assertions.*;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeSubsystem extends TestableSubsystem {
  /** Creates a new ExampleSubsystem. */

  private TalonFX intakeMotor;
  private WPI_TalonSRX deployMotor;

  public IntakeSubsystem() 
  {
    intakeMotor = new TalonFX(Constants.IntakeConstants.INTAKE_MOTOR_ID);
    deployMotor = new WPI_TalonSRX(Constants.IntakeConstants.DEPLOY_MOTOR_ID);
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

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
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
