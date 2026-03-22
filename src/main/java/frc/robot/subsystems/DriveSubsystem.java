package frc.robot.subsystems;

import org.fairportrobotics.frc.posty.TestableSubsystem;
import org.fairportrobotics.frc.posty.test.PostTest;
import static org.fairportrobotics.frc.posty.assertions.Assertions.*;
import org.fairportrobotics.frc.robolib.drivesystems.swerve.SwerveBuilder;
import org.fairportrobotics.frc.robolib.drivesystems.swerve.SwerveDriveSystem;
import org.fairportrobotics.frc.robolib.drivesystems.swerve.SwerveModule;
import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.annotation.JsonCreator.Mode;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants;

public class DriveSubsystem extends TestableSubsystem {

    private SendableChooser<Command> autoChooser;

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
                .withPigeonId(Constants.ExtraIDEntities.PIGEON_ID)
                .withMaxLinearVelocity(3.0)
                .withMaxAngularVelocity(Math.PI)
                .withSwerveModule(
                        swerveBuilder.new SwerveModuleBuilder()
                                .withDriveMotorId(Constants.SwerveDriveIDs.FRONT_LEFT_DRIVE_ID)
                                .withDriveKP(driveP)
                                .withDriveKI(driveI)
                                .withDriveKD(driveD)
                                .withDriveKV(driveKV)
                                .withSteerMotorId(Constants.SwerveDriveIDs.FRONT_LEFT_STEER_ID)
                                .withSteerKP(steerP)
                                .withSteerKI(steerI)
                                .withSteerKD(steerD)
                                .withSteerKV(steerKV)
                                // If the offsets don't work, contact Tyler W or Dom G
                                .withSteerOffset(Constants.SwerveDriveOffsets.FRONT_LEFT_MOTOR_OFFSET)
                                .withSteerEncoderId(Constants.SwerveDriveIDs.FRONT_LEFT_ENCODER_ID)
                                .withModuleLocation(new Translation2d(1, 1))
                                .withModuleName("Front Left")
                                .withGearRatio(8.14)
                                .withWheelDiameter(0.1016)
                                .build())
                .withSwerveModule(
                        swerveBuilder.new SwerveModuleBuilder()
                                .withDriveMotorId(Constants.SwerveDriveIDs.FRONT_RIGHT_DRIVE_ID)
                                .withDriveKP(driveP)
                                .withDriveKI(driveI)
                                .withDriveKD(driveD)
                                .withDriveKV(driveKV)
                                .withDriveInverted()
                                .withSteerMotorId(Constants.SwerveDriveIDs.FRONT_RIGHT_STEER_ID)
                                .withSteerKP(steerP)
                                .withSteerKI(steerI)
                                .withSteerKD(steerD)
                                .withSteerKV(steerKV)
                                .withSteerOffset(Constants.SwerveDriveOffsets.FRONT_RIGHT_MOTOR_OFFSET)
                                .withSteerEncoderId(Constants.SwerveDriveIDs.FRONT_RIGHT_ENCODER_ID)
                                .withModuleLocation(new Translation2d(1, -1))
                                .withModuleName("Front Right")
                                .withGearRatio(8.14)
                                .withWheelDiameter(0.1016)
                                .build())
                .withSwerveModule(
                        swerveBuilder.new SwerveModuleBuilder()
                                .withDriveMotorId(Constants.SwerveDriveIDs.BACK_LEFT_DRIVE_ID)
                                .withDriveKP(driveP)
                                .withDriveKI(driveI)
                                .withDriveKD(driveD)
                                .withDriveKV(driveKV)
                                .withSteerMotorId(Constants.SwerveDriveIDs.BACK_LEFT_STEER_ID)
                                .withSteerKP(steerP)
                                .withSteerKI(steerI)
                                .withSteerKD(steerD)
                                .withSteerKV(steerKV)
                                .withSteerOffset(Constants.SwerveDriveOffsets.BACK_LEFT_MOTOR_OFFSET)
                                .withSteerEncoderId(Constants.SwerveDriveIDs.BACK_LEFT_ENCODER_ID)
                                .withModuleLocation(new Translation2d(-1, 1))
                                .withModuleName("Back Left")
                                .withGearRatio(8.14)
                                .withWheelDiameter(0.1016)
                                .build())
                .withSwerveModule(
                        swerveBuilder.new SwerveModuleBuilder()
                                .withDriveMotorId(Constants.SwerveDriveIDs.BACK_RIGHT_DRIVE_ID)
                                .withDriveKP(driveP)
                                .withDriveKI(driveI)
                                .withDriveKD(driveD)
                                .withDriveKV(driveKV)
                                .withDriveInverted()
                                .withSteerMotorId(Constants.SwerveDriveIDs.BACK_RIGHT_STEER_ID)
                                .withSteerKP(steerP)
                                .withSteerKI(steerI)
                                .withSteerKD(steerD)
                                .withSteerKV(steerKV)
                                .withSteerOffset(Constants.SwerveDriveOffsets.BACK_RIGHT_MOTOR_OFFSET)
                                .withSteerEncoderId(Constants.SwerveDriveIDs.BACK_RIGHT_ENCODER_ID)
                                .withModuleLocation(new Translation2d(-1, -1))
                                .withModuleName("Back Right")
                                .withGearRatio(8.14)
                                .withWheelDiameter(0.1016)
                                .build())
                .build();

        this.setDefaultCommand(Commands.run(new Runnable() {

            @Override
            public void run() {
                driveSystem.setChassisSpeedsFromJoystickFieldRelative(
                        -(xboxController.getLeftY() * Math.abs(xboxController.getLeftY())),
                        -(xboxController.getLeftX() * Math.abs(xboxController.getLeftX())),
                        -(xboxController.getRawAxis(4) * Math.abs(xboxController.getRawAxis(4))));

                // driveSubsystem.setChassisSpeed(new ChassisSpeeds(1, 0, 0));

                Logger.recordOutput("RequestedSwerveState", driveSystem.getRequestedModuleStates());
                Logger.recordOutput("ActualSwerveState", driveSystem.getActualModuleStates());
                Logger.recordOutput("Pose Estimation", driveSystem.getRobotPose3d());
            }

        }, this));

        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                    driveSystem::getRobotPose2d,
                    driveSystem::setPose2d,
                    driveSystem::getRobotRelativeSpeeds,
                    (speeds, feedforwards) -> driveSystem.setChassisSpeed(speeds, new Translation2d()),
                    new PPHolonomicDriveController(
                            new PIDConstants(5.0, 0, 0),
                            new PIDConstants(5.0, 0, 0)),
                    config,
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this);

            autoChooser = AutoBuilder.buildAutoChooser();
            SmartDashboard.putData("Auto Chooser", autoChooser);

        } catch (Exception e) {
            e.printStackTrace();
        }

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

    public Command getAutoCommand(){
        return autoChooser.getSelected();
    }

    @PostTest()
    public void DriveSubsystem_CanDevicesConnected() {

        SwerveModule[] modules = driveSystem.getModules();

        for (int i = 0; i < modules.length; i++) {
            assertThat(modules[i].getDriveMotor().isConnected())
                    .as(modules[i].getModuleName() + " drive motor is not connected").isTrue();
            assertThat(modules[i].getSteerMotor().isConnected())
                    .as(modules[i].getModuleName() + " steer motor is not connected").isTrue();
            assertThat(modules[i].getSteerEncoder().isConnected())
                    .as(modules[i].getModuleName() + " CANCoder is not connected").isTrue();
        }

        assertThat(driveSystem.getGyro().isConnected()).as("Pigeon is not connected").isTrue();
    }

}
