// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import org.fairportrobotics.frc.posty.TestableSubsystem;
import static org.fairportrobotics.frc.posty.assertions.Assertions.assertThat;
import org.fairportrobotics.frc.posty.test.PostTest;
import static org.fairportrobotics.frc.robolib.motors.Utils.SetMotorDirection;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;

public class IntakeSubsystem extends TestableSubsystem {

    /**
     * Creates a new ExampleSubsystem.
     */

    private TalonFX intakeMotor;
    private WPI_TalonSRX deployMotor;

    public IntakeSubsystem() {
        intakeMotor = new TalonFX(Constants.IntakeConstants.INTAKE_MOTOR_ID);
        SetMotorDirection(intakeMotor, Constants.IntakeConstants.INTAKE_MOTOR_DIRECTION);

        deployMotor = new WPI_TalonSRX(Constants.IntakeConstants.DEPLOY_MOTOR_ID);
        deployMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        deployMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        deployMotor.setInverted(Constants.IntakeConstants.DEPLOY_MOTOR_INVERTED);
    }

    public Command deploy() {
        return Commands.sequence(
                Commands.deadline(
                        Commands.waitSeconds(2),
                        this.runOnce(
                                () -> {
                                    deployMotor.set(1);
                                } // Bump this speed up if intake doesn't deploy
                        )
                ),
                this.runOnce(() -> deployMotor.stopMotor())
        );
    }

    public Command resetDeploy() {
        return this.runEnd(() -> deployMotor.set(-1), () -> deployMotor.stopMotor());
    }

    public void setSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public Command stopIntake() {
        return this.runOnce(() -> {
            intakeMotor.stopMotor();
        });
    }

    public Command intake(XboxController m_XboxController) {
        return this.runOnce(() -> {
            if (intakeMotor.get() == 0) {
                setSpeed(-.5);
                m_XboxController.setRumble(RumbleType.kBothRumble, .5);
            } else {
                intakeMotor.stopMotor();
                m_XboxController.setRumble(RumbleType.kBothRumble, .0);
            }
        });
    }

    public Command reverseIntake() {
        return this.runEnd(() -> setSpeed(.5), () -> {
            intakeMotor.stopMotor();
        });
    }

    @PostTest(enabled = true)
    public void IntakeSubsystem_CANDevicesConnected() {
        assertThat(intakeMotor.isConnected()).as("Intake motor not connected!").isTrue();
        assertThat(deployMotor.isAlive()).as("Deploy controller not connected!").isTrue();
    }
}
