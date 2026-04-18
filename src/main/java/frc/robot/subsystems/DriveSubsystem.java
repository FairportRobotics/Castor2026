package frc.robot.subsystems;

import static org.fairportrobotics.frc.posty.assertions.Assertions.assertThat;

import org.fairportrobotics.frc.posty.TestableSubsystem;
import org.fairportrobotics.frc.posty.test.PostTest;
import org.fairportrobotics.frc.robolib.drivesystems.swerve.SwerveBuilder;
import org.fairportrobotics.frc.robolib.drivesystems.swerve.SwerveDriveSystem;
import org.fairportrobotics.frc.robolib.drivesystems.swerve.SwerveModule;
import org.fairportrobotics.frc.robolib.vision.limelight.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class DriveSubsystem extends TestableSubsystem {

    private SendableChooser<Command> autoChooser;

    private SwerveBuilder swerveBuilder = new SwerveBuilder();
    public SwerveDriveSystem driveSystem;

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
                .withMaxLinearVelocity(Constants.RobotChassisLimits.MAX_ROBOT_LINEAR_VELOCITY)
                .withMaxAngularVelocity(Constants.RobotChassisLimits.MAX_ROBOT_ANGULAR_VELOCITY)
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

        // Set vision measurement confidence values
        driveSystem.getPoseEstimator().setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 0.7, 0.1));

        this.setDefaultCommand(Commands.run(new Runnable() {

            @Override
            public void run() {
                driveSystem.setChassisSpeedsFromJoystickFieldRelative(
                        -(xboxController.getLeftY() * Math.abs(xboxController.getLeftY())),
                        -(xboxController.getLeftX() * Math.abs(xboxController.getLeftX())),
                        -(xboxController.getRawAxis(4) * Math.abs(xboxController.getRawAxis(4))));

                Logger.recordOutput("RequestedSwerveState", driveSystem.getRequestedModuleStates());
                Logger.recordOutput("ActualSwerveState", driveSystem.getActualModuleStates());
            }

        }, this));

        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                    driveSystem::getRobotPose2d,
                    driveSystem::setPose2d,
                    driveSystem::getRobotRelativeSpeeds,
                    (speeds, feedforwards) -> {driveSystem.setChassisSpeed(ChassisSpeeds.fromRobotRelativeSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, -speeds.omegaRadiansPerSecond, driveSystem.getRobotHeading()), new Translation2d());},
                    new PPHolonomicDriveController(
                            new PIDConstants(5, 0, 0),
                            new PIDConstants(Math.PI, 0, 0)),
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
            autoChooser.setDefaultOption("Do Nothing", Commands.none());
            SmartDashboard.putData("Auto Chooser", autoChooser);

            // PathPlannerLogging.setLogActivePathCallback((List<Pose2d> pose) -> {});
            PathPlannerLogging.setLogTargetPoseCallback((Pose2d targetPose) -> { Logger.recordOutput("PathPlanner-TargetPose", targetPose);});

        } catch (Exception e) {
            e.printStackTrace();
        }

        

    }

    @Override
    public void periodic() {
        super.periodic();
        driveSystem.periodic();

        Logger.recordOutput("RobotPose", driveSystem.getRobotPose3d());

        LimelightHelpers.SetRobotOrientation(Constants.CameraConstanst.FRONT_CAMERA_NAME, driveSystem.getRobotPose2d().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(Constants.CameraConstanst.BACK_CAMERA_NAME, driveSystem.getRobotPose2d().getRotation().getDegrees(), 0, 0, 0, 0, 0);

        // MegaTag 2 impl
        if(RobotState.isDisabled()){
            LimelightHelpers.SetIMUMode(Constants.CameraConstanst.FRONT_CAMERA_NAME, 1);
            LimelightHelpers.SetIMUMode(Constants.CameraConstanst.BACK_CAMERA_NAME, 1);
        }else{
            LimelightHelpers.SetIMUMode(Constants.CameraConstanst.FRONT_CAMERA_NAME, 4);
            LimelightHelpers.SetIMUMode(Constants.CameraConstanst.BACK_CAMERA_NAME, 4);
        }

        LimelightHelpers.PoseEstimate frontTag = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.CameraConstanst.FRONT_CAMERA_NAME);
        LimelightHelpers.PoseEstimate backTag = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.CameraConstanst.BACK_CAMERA_NAME);

        if(frontTag.tagCount >= 2){
            driveSystem.getPoseEstimator().addVisionMeasurement(new Pose3d(frontTag.pose), frontTag.timestampSeconds);
            Logger.recordOutput("FrontPose-Estimate", frontTag.pose);
            Logger.recordOutput("FrontPose-IsMegaTag2", frontTag.isMegaTag2);
        }

        if(backTag.tagCount >= 2){
            driveSystem.getPoseEstimator().addVisionMeasurement(new Pose3d(backTag.pose), backTag.timestampSeconds);
            Logger.recordOutput("BackPose-Estimate", backTag.pose);
            Logger.recordOutput("BackPose-IsMegaTag2", frontTag.isMegaTag2);
        }


        // MegaTag 1 Impl
        // LimelightHelpers.PoseEstimate frontCameraPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.CameraConstanst.FRONT_CAMERA_NAME);
        // if (frontCameraPose.tagCount >= 2) {
        //     driveSystem.getPoseEstimator().addVisionMeasurement(new Pose3d(frontCameraPose.pose), frontCameraPose.timestampSeconds);
        //     Logger.recordOutput("FrontPoseEstimate", frontCameraPose.pose);
        // }
        // LimelightHelpers.PoseEstimate backCameraPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.CameraConstanst.BACK_CAMERA_NAME);
        // if (backCameraPose.tagCount >= 2) {
        //     driveSystem.getPoseEstimator().addVisionMeasurement(new Pose3d(backCameraPose.pose), backCameraPose.timestampSeconds);
        //     Logger.recordOutput("BackPoseEstimate", backCameraPose.pose);
        // }
    }

    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();
        driveSystem.simulationPeriodic();
    }

    public Command getAutoCommand() {
        return autoChooser.getSelected();
    }

    public void stopDrive() {
        driveSystem.setChassisSpeed(ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, 0, driveSystem.getGyro().getRotation2d()), Translation2d.kZero);
    }

    public void rotateChassis(double rotSpeed) {
        driveSystem.setChassisSpeed(ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, rotSpeed, driveSystem.getGyro().getRotation2d()), Translation2d.kZero);
    }

    public Pose3d getBotPose() {
        return driveSystem.getRobotPose3d();
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
