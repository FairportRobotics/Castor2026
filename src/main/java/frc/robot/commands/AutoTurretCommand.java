package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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
import frc.robot.subsystems.TurretSubsystem;

public class AutoTurretCommand extends Command {

    private final Translation2d CENTER_TURRET_TO_ROBOT = new Translation2d(-0.1335024, -0.1824736);

    private TurretSubsystem turretSubsystem;
    private DriveSubsystem driveSubsystem;

    private int[] shootingTagFilters;

    private AprilTagFieldLayout fieldTags = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    private Pose3d closestTag;
    private Pose3d scoringHubPose;

    public AutoTurretCommand(TurretSubsystem turretSubsystem, DriveSubsystem driveSubsystem) {
        this.turretSubsystem = turretSubsystem;
        this.driveSubsystem = driveSubsystem;

        addRequirements(turretSubsystem); // Not adding drive as we don't wan't the drive command to be de-scheduled
    }

    @Override
    public void initialize() {
        // Perform auto-home
        turretSubsystem.homeTurret();

        DriverStation.getAlliance().ifPresent((aliance) -> {
            if (aliance == Alliance.Blue) {
                shootingTagFilters = Constants.CameraConstanst.IDFilters.BLUE_HUB_SHOOTING_IDS;

                scoringHubPose = fieldTags.getTagPose(26).get();
                scoringHubPose = scoringHubPose.transformBy(new Transform3d(new Transform2d(-0.5207, 0, Rotation2d.kZero)));
            } else {
                shootingTagFilters = Constants.CameraConstanst.IDFilters.RED_HUB_SHOOTING_IDS;

                scoringHubPose = fieldTags.getTagPose(10).get();
                scoringHubPose = scoringHubPose.transformBy(new Transform3d(new Transform2d(-0.5207, 0, Rotation2d.kZero)));
            }
            Logger.recordOutput("AutoTurret-ScoringTarget", scoringHubPose);
        });

    }

    @Override
    public void execute() {

        Pose3d botPose = driveSubsystem.getBotPose();
        Pose3d turretPose = botPose.plus(new Transform3d(new Translation3d(CENTER_TURRET_TO_ROBOT),
                new Rotation3d(Rotation2d.fromRotations(turretSubsystem.getTurretAngle().in(Units.Rotations)))));
        Logger.recordOutput("AutoTurret-TurretPose", turretPose);

        Translation2d targetTranslation = scoringHubPose.getTranslation().toTranslation2d().minus(turretPose.getTranslation().toTranslation2d());
        Logger.recordOutput("AutoTurret-Translation", targetTranslation);


        if(turretSubsystem.isTurretReady()){
            turretSubsystem.setTurretRotation(botPose.getRotation().toRotation2d().minus(targetTranslation.getAngle()).plus(Rotation2d.k180deg).getMeasure());
            // turretSubsystem.setTurretRotation(Angle.ofRelativeUnits(-180, Units.Degrees));
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }

}
