package frc.robot.commands;

import java.util.Arrays;
import java.util.List;

import org.fairportrobotics.frc.robolib.vision.limelight.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AutoShootCommandChassisTurretPathPlanner extends Command {

    // For Chassis Control
    private DriveSubsystem driveSubsystem;

    private HopperSubsystem hopperSubsystem;
    private TurretSubsystem turretSubsystem;

    private int[] tagFilters;

    private AprilTagFieldLayout fieldTags = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    private Pose3d closestTag;
    private Pose2d targetPPPose;

    private Command pathFollowCommand;

    public AutoShootCommandChassisTurretPathPlanner(DriveSubsystem driveSubsystem, HopperSubsystem hopperSubsystem,
            TurretSubsystem turretSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.hopperSubsystem = hopperSubsystem;
        this.turretSubsystem = turretSubsystem;

        this.addRequirements(driveSubsystem, hopperSubsystem, turretSubsystem);

    }

    @Override
    public void initialize() {

        DriverStation.getAlliance().ifPresent((aliance) -> {
            Pose3d targetHub;
            if (aliance == Alliance.Blue) {
                tagFilters = Constants.CameraConstanst.IDFilters.BLUE_HUB_SHOOTING_IDS;
                targetHub = Constants.FieldPoses.BLUE_HUB_POSE;
            } else {
                tagFilters = Constants.CameraConstanst.IDFilters.RED_HUB_SHOOTING_IDS;
                targetHub = Constants.FieldPoses.RED_HUB_POSE;
            }

            Pose3d botPose = driveSubsystem.getBotPose();
            // Find closest target to us
            List<Pose3d> tagPoses = Arrays.stream(tagFilters).mapToObj(tagId -> fieldTags.getTagPose(tagId).get())
                    .toList();
            closestTag = botPose.nearest(tagPoses);
            Logger.recordOutput("AutoAlignPP-ClosestAprilTagPose", closestTag);

            Translation2d delta = botPose.getTranslation().toTranslation2d()
                    .minus(closestTag.getTranslation().toTranslation2d());
            Logger.recordOutput("AutoAlignPP-ClosestAprilTagTransform", delta);

            // Add 180 because we want the back of the robot to face the HUB
            // deadreckoningAutoCenterController.setSetpoint(delta.getAngle().plus(Rotation2d.k180deg).getRadians());

            // Translate back 1 meter from HUB
            Translation2d newTrans = targetHub.getTranslation().toTranslation2d()
                    .plus(new Translation2d(1.5, delta.getAngle()));
            targetPPPose = new Pose2d(newTrans, delta.getAngle());
            Logger.recordOutput("AutoAlignPP-PPTargetPose", targetPPPose);

            pathFollowCommand = AutoBuilder.pathfindToPose(targetPPPose,
                    new PathConstraints(Constants.RobotChassisLimits.MAX_ROBOT_LINEAR_VELOCITY,
                            Constants.RobotChassisLimits.MAX_ROBOT_LINEAR_VELOCITY_ACCEL,
                            Constants.RobotChassisLimits.MAX_ROBOT_ANGULAR_VELOCITY,
                            Constants.RobotChassisLimits.MAX_ROBOT_LINEAR_VELOCITY_ACCEL));

            CommandScheduler.getInstance().schedule(pathFollowCommand);
        });

        turretSubsystem.setLauncher(5500);
        hopperSubsystem.spindexerOn();

    }

    @Override
    public void execute() {

        // If robot is centered on HUB and shooter is up to speed. Run the kicker
        if (turretSubsystem.isLauncherUpToSpeed() && pathFollowCommand.isFinished()) {
            Logger.recordOutput("AutoAlignState-Shoot", "FIRE!");
            hopperSubsystem.feedKicker();
        } else {
            Logger.recordOutput("AutoAlignState-Shoot", "Waiting");
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        pathFollowCommand.cancel();
        turretSubsystem.setLauncher(0);

        hopperSubsystem.stopKicker();
        hopperSubsystem.spindexerOff();

        driveSubsystem.stopDrive();

        LimelightHelpers.setPipelineIndex(Constants.CameraConstanst.BACK_CAMERA_NAME,
                Constants.CameraConstanst.BACK_LOCALIZATION_PIPELINE_NUMBER);
    }

}
