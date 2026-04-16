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
import frc.robot.Constants.ShootingRegion;
import frc.robot.Utils;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AutoTurretCommand extends Command {

    private final Translation2d CENTER_TURRET_TO_ROBOT = new Translation2d(-0.1335024, -0.1824736);

    private TurretSubsystem turretSubsystem;
    private DriveSubsystem driveSubsystem;

    private Alliance alliance;

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
                turretSubsystem.setTurretTargetPose(Constants.FieldPoses.BLUE_HUB_POSE);
                aliance = Alliance.Blue;
            } else {
                turretSubsystem.setTurretTargetPose(Constants.FieldPoses.RED_HUB_POSE);
                aliance = Alliance.Red;
            }
        });

    }

    @Override
    public void execute() {

        ShootingRegion shootingRegion = Utils.findRobotShootingRegion(driveSubsystem.getBotPose(), DriverStation.getAlliance().get());

        if(shootingRegion != ShootingRegion.NON_SHOOTING_REGION){
            Pose3d targetPose = Utils.findShootingTarget(shootingRegion, DriverStation.getAlliance().get());

            turretSubsystem.setTurretTargetPose(targetPose);
        }

        Logger.recordOutput("AutoTurret-ScoringTarget", turretSubsystem.getTurretTargetPose());

        Pose3d botPose = driveSubsystem.getBotPose();
        Pose3d turretPose = botPose.plus(new Transform3d(new Translation3d(CENTER_TURRET_TO_ROBOT),
                new Rotation3d(Rotation2d.fromRotations(turretSubsystem.getTurretAngleRobotRelative() + 0.5))));
        Logger.recordOutput("AutoTurret-TurretPose", turretPose);

        Translation2d targetTranslation = turretSubsystem.getTurretTargetPose().getTranslation().toTranslation2d().minus(turretPose.getTranslation().toTranslation2d());
        Logger.recordOutput("AutoTurret-Translation", targetTranslation);

        if(turretSubsystem.isTurretReady()){
            turretSubsystem.setTurretFieldRelative(botPose, targetTranslation.getAngle().getRotations());
            // turretSubsystem.setTurretRobotRelative(0.25);
            // turretSubsystem.setTurretMotorRotation(0);
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
