package frc.robot.commands;

import java.util.Arrays;
import java.util.List;

import org.fairportrobotics.frc.robolib.vision.limelight.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AutoShootCommandChassisTurret extends Command{
    
    // For Chassis Control
    private DriveSubsystem driveSubsystem;
    
    private HopperSubsystem hopperSubsystem;
    private TurretSubsystem turretSubsystem;

    private PIDController cameraAutoCenterController;
    private PIDController deadreckoningAutoCenterController;

    private int[] tagFilters;

    private AprilTagFieldLayout fieldTags = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    private Pose3d closestTag;

    private final double chassisRotateSpeed = Math.PI * 0.1;

    public AutoShootCommandChassisTurret(DriveSubsystem driveSubsystem, HopperSubsystem hopperSubsystem, TurretSubsystem turretSubsystem)
    {
        this.driveSubsystem = driveSubsystem;
        this.hopperSubsystem = hopperSubsystem;
        this.turretSubsystem = turretSubsystem;

        this.addRequirements(driveSubsystem, hopperSubsystem, turretSubsystem);

        cameraAutoCenterController = new PIDController(0.1, 0, 0.0);
        cameraAutoCenterController.setTolerance(0.2); // Units is meters
        cameraAutoCenterController.setSetpoint(0);


        deadreckoningAutoCenterController = new PIDController(Math.PI * 0.1, 0, 0.0);
        deadreckoningAutoCenterController.setTolerance(0.2); // Units is rotations
        deadreckoningAutoCenterController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        DriverStation.getAlliance().ifPresent((aliance) -> {
            if (aliance == Alliance.Blue) {
                tagFilters = Constants.CameraConstanst.IDFilters.BLUE_HUB_SHOOTING_IDS;
            } else {
                tagFilters = Constants.CameraConstanst.IDFilters.RED_HUB_SHOOTING_IDS;
            }
        });

        turretSubsystem.setLauncher(5500);
        hopperSubsystem.spindexerOn();
        cameraAutoCenterController.calculate(-100);

        // Change to HUB tracking pipeline
        LimelightHelpers.setPipelineIndex(Constants.CameraConstanst.BACK_CAMERA_NAME, Constants.CameraConstanst.BACK_HUB_TRACKING_2D_PIPELINE_NUMBER);
        LimelightHelpers.SetFiducialIDFiltersOverride(Constants.CameraConstanst.BACK_CAMERA_NAME, tagFilters);

        Pose3d botPose = driveSubsystem.getBotPose();
        // Find closest target to us
        List<Pose3d> tagPoses = Arrays.stream(tagFilters).mapToObj(tagId -> fieldTags.getTagPose(tagId).get()).toList();
        closestTag = botPose.nearest(tagPoses);
        Logger.recordOutput("AutoAlign-ClosestAprilTagPose", closestTag);

        Translation2d delta = botPose.getTranslation().toTranslation2d().minus(closestTag.getTranslation().toTranslation2d());
        Logger.recordOutput("AutoAlign-ClosestAprilTagTransform", delta);

        deadreckoningAutoCenterController.setSetpoint(delta.getAngle().plus(Rotation2d.k180deg).getRadians());
    }

    @Override
    public void execute() {
        // Need to compute shoot speed and angle

        Logger.recordOutput("AutoAlign-CenteringError", cameraAutoCenterController.getError());

        // No april tag in camera view. Rotate towards our theoretical closest tag
        if(!LimelightHelpers.getTV(Constants.CameraConstanst.BACK_CAMERA_NAME)){ 
            Logger.recordOutput("AutoAlignState", "Look for tag");

            Pose3d botPose = driveSubsystem.getBotPose();
            //driveSubsystem.rotateChassis(deadreckoningAutoCenterController.calculate(botPose.getRotation().toRotation2d().getRadians())); // May need to flip this
        }
        else // We have an april tag, center it to the camera frame
        {
            Logger.recordOutput("AutoAlignState", "Center on tag");
            // [tx, ty, tz, pitch, yaw, roll]
            double[] targetsPose = LimelightHelpers.getTargetPose_CameraSpace(Constants.CameraConstanst.BACK_CAMERA_NAME);
            if(targetsPose.length >= 1){
                driveSubsystem.rotateChassis(-cameraAutoCenterController.calculate(targetsPose[0])); // This may need to be negated
            }
        }

        // If robot is centered on HUB and shooter is up to speed. Run the kicker
        if(turretSubsystem.isLauncherUpToSpeed() && cameraAutoCenterController.atSetpoint()){
            Logger.recordOutput("AutoAlignState", "FIRE!");
            hopperSubsystem.feedKicker();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.setLauncher(0);

        hopperSubsystem.stopKicker();
        hopperSubsystem.spindexerOff();

        driveSubsystem.stopDrive();

        LimelightHelpers.setPipelineIndex(Constants.CameraConstanst.BACK_CAMERA_NAME, Constants.CameraConstanst.BACK_LOCALIZATION_PIPELINE_NUMBER);
    }

}
