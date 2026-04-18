// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import org.fairportrobotics.frc.posty.TestableSubsystem;
import static org.fairportrobotics.frc.posty.assertions.Assertions.assertThat;
import org.fairportrobotics.frc.posty.test.PostTest;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class HopperSubsystem extends TestableSubsystem {

    private TalonFX spindexerMotor;

    private TalonFX kickerMotor;
    private VelocityVoltage kickerControl = new VelocityVoltage(0);
    private StatusSignal<AngularVelocity> kickerVelocity;

    /**
     * Creates a new HopperSubsystem.
     */
    public HopperSubsystem() {
        spindexerMotor = new TalonFX(Constants.HopperConstants.SPINDEXER_MOTOR_ID);
        spindexerMotor.getConfigurator().apply(
                new TalonFXConfiguration()
                        .withMotorOutput(
                                new MotorOutputConfigs()
                                        .withInverted(Constants.HopperConstants.SPINDEXER_MOTOR_DIRECTION))
                        .withSlot0(
                                new Slot0Configs()
                                        .withKP(0)
                                        .withKI(0)
                                        .withKD(0)
                                        .withKS(0)));

        kickerMotor = new TalonFX(Constants.HopperConstants.KICKER_MOTOR_ID);
        kickerMotor.getConfigurator().apply(
                new TalonFXConfiguration()
                        .withMotorOutput(
                                new MotorOutputConfigs()
                                        .withInverted(Constants.HopperConstants.KICKER_MOTOR_DIRECTION)
                        )
                        .withSlot0(new Slot0Configs()
                                .withKP(0)
                                .withKI(0)
                                .withKD(0)
                                .withKS(0)
                        ));
        kickerVelocity = kickerMotor.getVelocity();
    }

    public void feedKicker() {
        //kickerMotor.setControl(kickerControl.withVelocity(1000));
        kickerMotor.set(0.75);
    }

    public void reverseKicker() {
        //kickerMotor.setControl(kickerControl.withVelocity(-500));
        kickerMotor.set(-0.5);
    }

    public void stopKicker() {
        kickerMotor.stopMotor();
    }

    public void spindexerOn() {
        spindexerMotor.set(0.75);
    }

    public void spindexerReverse() {
        spindexerMotor.set(-0.75);
    }

    public void spindexerOff() {
        spindexerMotor.stopMotor();
    }

    public Command spinny() {
        return this.runOnce(() -> {
            if (spindexerMotor.get() == 0) {
                spindexerOn();
            } else {
                spindexerOff();
            }
        });
    }


    @Override
    public void periodic() {
        Logger.recordOutput("HopperSubsystem-KickerSpeed(Rotations/sec)", kickerVelocity.refresh().getValue().in(Units.RotationsPerSecond));
    }

    @PostTest(enabled = true)
    public void hopperSystem_CANDevicesConnected() {
        assertThat(kickerMotor.isConnected()).as("Kicker motor not connected!").isTrue();
        assertThat(spindexerMotor.isConnected()).as("Spindexer motor not connected!").isTrue();
    }

}
