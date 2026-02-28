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
                .withPigeonId(1)
                .withMaxLinearVelocity(3.0)
                .withMaxAngularVelocity(Math.PI * 2)
                .withSwerveModule(
                        swerveBuilder.new SwerveModuleBuilder()
                                .withDriveMotorId(12)
                                .withDriveKP(driveP)
                                .withDriveKI(driveI)
                                .withDriveKD(driveD)
                                .withDriveKV(driveKV)
                                .withSteerMotorId(11)
                                .withSteerKP(steerP)
                                .withSteerKI(steerI)
                                .withSteerKD(steerD)
                                .withSteerKV(steerKV)
                                .withSteerOffset(-0.338623)
                                .withSteerEncoderId(13)
                                .withModuleLocation(new Translation2d(1, 1))
                                .withModuleName("Front Left")
                                .withGearRatio(8.14)
                                .withWheelDiameter(0.1016)
                                .build())
                .withSwerveModule(
                        swerveBuilder.new SwerveModuleBuilder()
                                .withDriveMotorId(22)
                                .withDriveKP(driveP)
                                .withDriveKI(driveI)
                                .withDriveKD(driveD)
                                .withDriveKV(driveKV)
                                .withDriveInverted()
                                .withSteerMotorId(21)
                                .withSteerKP(steerP)
                                .withSteerKI(steerI)
                                .withSteerKD(steerD)
                                .withSteerKV(steerKV)
                                .withSteerOffset(0.110840)
                                .withSteerEncoderId(23)
                                .withModuleLocation(new Translation2d(1, -1))
                                .withModuleName("Front Right")
                                .withGearRatio(8.14)
                                .withWheelDiameter(0.1016)
                                .build())
                .withSwerveModule(
                        swerveBuilder.new SwerveModuleBuilder()
                                .withDriveMotorId(32)
                                .withDriveKP(driveP)
                                .withDriveKI(driveI)
                                .withDriveKD(driveD)
                                .withDriveKV(driveKV)
                                .withSteerMotorId(31)
                                .withSteerKP(steerP)
                                .withSteerKI(steerI)
                                .withSteerKD(steerD)
                                .withSteerKV(steerKV)
                                .withSteerOffset(-0.496582)
                                .withSteerEncoderId(33)
                                .withModuleLocation(new Translation2d(-1, 1))
                                .withModuleName("Back Left")
                                .withGearRatio(8.14)
                                .withWheelDiameter(0.1016)
                                .build())
                .withSwerveModule(
                        swerveBuilder.new SwerveModuleBuilder()
                                .withDriveMotorId(42)
                                .withDriveKP(driveP)
                                .withDriveKI(driveI)
                                .withDriveKD(driveD)
                                .withDriveKV(driveKV)
                                .withDriveInverted()
                                .withSteerMotorId(41)
                                .withSteerKP(steerP)
                                .withSteerKI(steerI)
                                .withSteerKD(steerD)
                                .withSteerKV(steerKV)
                                .withSteerOffset(0.463379)
                                .withSteerEncoderId(43)
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
