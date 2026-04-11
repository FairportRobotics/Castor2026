package frc.robot.commands;

import org.fairportrobotics.frc.robolib.vision.limelight.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class ZoneCheck extends Command {
    private Zone zone;
    private DriveSubsystem driveSubsystem;

    public ZoneCheck(DriveSubsystem driveSubsystem)
    {
        this.driveSubsystem = driveSubsystem;
    }

    public enum Zone {
        Blue,
        Red,
        Neutral,
        INIT;
    }

    @Override
    public void initialize() {
        zone = Zone.INIT;
    }

    @Override
    public void execute() {
        getZone();
        Logger.recordOutput("FieldZone", zone);
        DriverStation.getAlliance().ifPresent((aliance) -> {
            if (aliance == Alliance.Blue) {
                switch (zone) {
                    case Blue:
                        System.out.println("BLUE");
                    case Neutral:
                        System.out.println("NEUTRAL");
                    case Red:
                        System.out.println("RED");
                    default:
                        System.out.println("****");
                }
            } else {
                switch (zone) {
                    case Blue:
                        System.out.println("BLUE");
                    case Neutral:
                        System.out.println("NEUTRAL");
                    case Red:
                        System.out.println("RED");
                    default:
                        System.out.println("****");
                }
            }
        });
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private void getZone() {
        if (driveSubsystem.getBotPose().getX() >= Constants.CameraConstanst.RED_ZONE_COOR[0]) {
            zone = Zone.Red;
        } else if (driveSubsystem.getBotPose().getX() >= Constants.CameraConstanst.NEUTRAL_ZONE_COOR[0]) {
            zone = Zone.Neutral;
        } else {
            zone = Zone.Blue;
        }
    }
}
