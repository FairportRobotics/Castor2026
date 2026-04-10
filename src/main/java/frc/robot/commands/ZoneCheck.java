package frc.robot.commands;

import org.fairportrobotics.frc.robolib.vision.limelight.LimelightHelpers;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class ZoneCheck extends Command {

    private AprilTagFieldLayout fieldTags = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    private Zone zone;

    public enum Zone {
        Blue,
        Red,
        Neutral;
    }

    public ZoneCheck() {
        getBotPose();
    }

    @Override
    public void initialize() {
        DriverStation.getAlliance().ifPresent((aliance) -> {
            if (aliance == Alliance.Blue) {
                switch (zone) {
                    case Blue:
                        System.out.println("Im Home");
                    case Neutral:
                        System.out.println("Uhhhhhhhh not red");
                        ;
                    default:
                        System.out.println("BOOOOO, BOOOO RED");
                        ;
                }
            } else {
                switch (zone) {
                    case Blue:
                        System.out.println("Screw Blue");
                        ;
                    case Neutral:
                        System.out.println("Uhhhhhhhh not red");
                        ;
                    default:
                        System.out.println("Im Home");
                }
            }
        });
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private double[] getBotPose() {
        return LimelightHelpers.getBotPose_wpiBlue(Constants.CameraConstanst.BACK_CAMERA_NAME);
    }

    private void getZone() {
        if (getBotPose()[0] >= Constants.CameraConstanst.RED_ZONE_COOR[0]) {
            zone = Zone.Red;
        } else if (getBotPose()[0] >= Constants.CameraConstanst.NEUTRAL_ZONE_COOR[0]) {
            zone = Zone.Neutral;
        } else {
            zone = Zone.Blue;
        }
    }
}
