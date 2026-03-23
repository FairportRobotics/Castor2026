package frc.robot.commands;

import java.util.Arrays;
import java.util.List;

import org.fairportrobotics.frc.robolib.vision.limelight.LimelightHelpers;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
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

    private PIDController autoCenterController;

    private int[] tagFilters;

    private AprilTagFieldLayout filedTags = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    private final double chassisRotateSpeed = Math.PI * 0.1;

    public AutoShootCommandChassisTurret(DriveSubsystem driveSubsystem, HopperSubsystem hopperSubsystem, TurretSubsystem turretSubsystem)
    {
        this.driveSubsystem = driveSubsystem;
        this.hopperSubsystem = hopperSubsystem;
        this.turretSubsystem = turretSubsystem;

        this.addRequirements(driveSubsystem, hopperSubsystem, turretSubsystem);

        autoCenterController = new PIDController(0.1, 0, 0);
        autoCenterController.setTolerance(1); // Units is pixels
        autoCenterController.setSetpoint(0);
 
        tagFilters = DriverStation.getAlliance().get() == Alliance.Blue ? Constants.CameraConstanst.IDFilters.BLUE_HUB_SHOOTING_IDS : Constants.CameraConstanst.IDFilters.RED_HUB_SHOOTING_IDS;
    }

    @Override
    public void initialize() {
        turretSubsystem.setLauncher(2500);
        hopperSubsystem.spindexerOn();

        // Change to HUB tracking pipeline
        LimelightHelpers.setPipelineIndex(Constants.CameraConstanst.BACK_CAMERA_NAME, Constants.CameraConstanst.BACK_HUB_TRACKING_2D_PIPELINE_NUMBER);
        LimelightHelpers.SetFiducialIDFiltersOverride(Constants.CameraConstanst.BACK_CAMERA_NAME, tagFilters);

    }

    @Override
    public void execute() {
        // Need to compute shoot speed and angle

        // No april tag in camera view. Rotate towards our theoretical closest tag
        if(!LimelightHelpers.getTV(Constants.CameraConstanst.BACK_CAMERA_NAME)){
            Pose3d botPose = driveSubsystem.getBotPose();
            // Find closest target to us
            List<Pose3d> tagPoses = Arrays.stream(tagFilters).mapToObj(tagId -> filedTags.getTagPose(tagId).get()).toList();
            for(int i=0;i<tagFilters.length; i++)
            {
                Pose3d closet = botPose.nearest(tagPoses);
                Transform3d delta = botPose.minus(closet);
                driveSubsystem.rotateChassis(delta.getRotation().getAngle() > 0 ? chassisRotateSpeed : -chassisRotateSpeed); // May need to flip this
            }
        }
        else // We have an april tag, center it to the camera frame
        {
            // [tx, ty, tz, pitch, yaw, roll]
            double[] targetsPose = LimelightHelpers.getTargetPose_CameraSpace(Constants.CameraConstanst.BACK_CAMERA_NAME);

            driveSubsystem.rotateChassis(autoCenterController.calculate(targetsPose[0])); // This may need to be negated
        }

        // If robot is centered on HUB and shooter is up to speed. Run the kicker
        if(turretSubsystem.isLauncherUpToSpeed() && autoCenterController.atSetpoint()){
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
