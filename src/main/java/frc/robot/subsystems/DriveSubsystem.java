package frc.robot.subsystems;

import org.fairportrobotics.frc.posty.TestableSubsystem;
import org.fairportrobotics.frc.posty.test.PostTest;
import static org.fairportrobotics.frc.posty.assertions.Assertions.*;
import org.fairportrobotics.frc.robolib.drivesystems.swerve.SwerveBuilder;
import org.fairportrobotics.frc.robolib.drivesystems.swerve.SwerveDriveSystem;
import org.fairportrobotics.frc.robolib.drivesystems.swerve.SwerveModule;
import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.annotation.JsonCreator.Mode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriveSubsystem extends TestableSubsystem {

    private SwerveBuilder swerveBuilder = new SwerveBuilder();
    private SwerveDriveSystem driveSystem;

    private double driveP = 0.1;
    private double driveI = 0.0;
    private double driveD = 0.01;
    private double driveKV = 0.2;

    private double steerP = 20;
    private double steerI = 0;
    private double steerD = 0.0;
    private double steerKV = 0;

    public DriveSubsystem(CommandXboxController xboxController) {

        driveSystem = swerveBuilder
                .withCanbusName("Drive")
                .withPigeonId(20)
                .withMaxLinearVelocity(2.5)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
                .withMaxAngularVelocity(Math.PI)
                .withSwerveModule(
                        swerveBuilder.new SwerveModuleBuilder()
                                .withDriveMotorId(12)
                                .withDriveKP(driveP)
                                .withDriveKI(driveI)
                                .withDriveKD(driveD)
                                .withDriveKV(driveKV)
                                .withSteerMotorId(3)
                                .withSteerKP(steerP)
                                .withSteerKI(steerI)
                                .withSteerKD(steerD)
                                .withSteerKV(steerKV)
                                .withSteerOffset(1.752197)
                                .withSteerEncoderId(10)
                                .withModuleLocation(new Translation2d(1, 1))
                                .withModuleName("Front Left")
                                .withGearRatio(8.14)
                                .withWheelDiameter(0.1016)
                                .build())
                .withSwerveModule(
                        swerveBuilder.new SwerveModuleBuilder()
                                .withDriveMotorId(5)
                                .withDriveKP(driveP)
                                .withDriveKI(driveI)
                                .withDriveKD(driveD)
                                .withDriveKV(driveKV)
                                .withDriveInverted()
                                .withSteerMotorId(6)
                                .withSteerKP(steerP)
                                .withSteerKI(steerI)
                                .withSteerKD(steerD)
                                .withSteerKV(steerKV)
                                .withSteerOffset(0.406494)
                                .withSteerEncoderId(1)
                                .withModuleLocation(new Translation2d(1, -1))
                                .withModuleName("Front Right")
                                .withGearRatio(8.14)
                                .withWheelDiameter(0.1016)
                                .build())
                .withSwerveModule(
                        swerveBuilder.new SwerveModuleBuilder()
                                .withDriveMotorId(8)
                                .withDriveKP(driveP)
                                .withDriveKI(driveI)
                                .withDriveKD(driveD)
                                .withDriveKV(driveKV)
                                .withSteerMotorId(2)
                                .withSteerKP(steerP)
                                .withSteerKI(steerI)
                                .withSteerKD(steerD)
                                .withSteerKV(steerKV)
                                .withSteerOffset(0.211670)
                                .withSteerEncoderId(7)
                                .withModuleLocation(new Translation2d(-1, 1))
                                .withModuleName("Back Left")
                                .withGearRatio(8.14)
                                .withWheelDiameter(0.1016)
                                .build())
                .withSwerveModule(
                        swerveBuilder.new SwerveModuleBuilder()
                                .withDriveMotorId(9)
                                .withDriveKP(driveP)
                                .withDriveKI(driveI)
                                .withDriveKD(driveD)
                                .withDriveKV(driveKV)
                                .withDriveInverted()
                                .withSteerMotorId(11)
                                .withSteerKP(steerP)
                                .withSteerKI(steerI)
                                .withSteerKD(steerD)
                                .withSteerKV(steerKV)
                                .withSteerOffset(-0.235107)
                                .withSteerEncoderId(4)
                                .withModuleLocation(new Translation2d(-1, -1))
                                .withModuleName("Back Right")
                                .withGearRatio(8.14)
                                .withWheelDiameter(0.1016)
                                .build())
                .build();

        this.setDefaultCommand(Commands.run(new Runnable() {

            @Override
            public void run() {
                driveSystem.setChassisSpeedsFromJoystickRobotRelative(
                        -(xboxController.getLeftY() * Math.abs(xboxController.getLeftY())),
                        -(xboxController.getLeftX() * Math.abs(xboxController.getLeftX())),
                        -(xboxController.getRawAxis(4) * Math.abs(xboxController.getRawAxis(4))));

                // driveSubsystem.setChassisSpeed(new ChassisSpeeds(1, 0, 0));

                Logger.recordOutput("RequestedSwerveState", driveSystem.getRequestedModuleStates());
                Logger.recordOutput("ActualSwerveState", driveSystem.getActualModuleStates());
                Logger.recordOutput("Pose Estimation", driveSystem.getRobotPose());
            }

        }, this));

    }

    @Override
    public void periodic() {
        super.periodic();
        driveSystem.periodic();
    }

    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();
        driveSystem.simulationPeriodic();
    }

    @PostTest()
    public void DriveSubsystem_CanDevicesConnected(){

        SwerveModule[] modules = driveSystem.getModules();

        for(int i=0;i < modules.length; i++){
            assertThat(modules[i].getDriveMotor().isConnected()).as(modules[i].getModuleName() + " drive motor is not connected").isTrue();
            assertThat(modules[i].getSteerMotor().isConnected()).as(modules[i].getModuleName() + " steer motor is not connected").isTrue();
            assertThat(modules[i].getSteerEncoder().isConnected()).as(modules[i].getModuleName() + " CANCoder is not connected").isTrue();
        }

        assertThat(driveSystem.getGyro().isConnected()).as("Pigeon is not connected").isTrue();

    }

}
