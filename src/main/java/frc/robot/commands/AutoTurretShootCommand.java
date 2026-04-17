package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AutoTurretShootCommand extends Command{

    private HopperSubsystem hopperSubsystem;
    private TurretSubsystem turretSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private DriveSubsystem driveSubsystem;

    private Command waitCommand = Commands.waitSeconds(1.5);

    private Alliance alliance;
    public AutoTurretShootCommand(HopperSubsystem hopperSubsystem, TurretSubsystem turretSubsystem, IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem){
        this.hopperSubsystem = hopperSubsystem;
        this.turretSubsystem = turretSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.driveSubsystem = driveSubsystem;

        addRequirements(hopperSubsystem, intakeSubsystem);
    }
    
    @Override
    public void initialize() {
        hopperSubsystem.spindexerOn();
        CommandScheduler.getInstance().schedule(waitCommand);
        alliance = DriverStation.getAlliance().get();
        Logger.recordOutput("AutoTurretShoot-State", "SHOOTING");
    }

    @Override
    public void execute() {
        double distance = turretSubsystem.getTurretTargetPose().toPose2d().getTranslation().getDistance(driveSubsystem.getBotPose().getTranslation().toTranslation2d());
        Logger.recordOutput("AutoTurretShoot-DistanceToTarget(Meters)", distance);

        turretSubsystem.setTargetElevation(Utils.getHoodAngleForDistance(distance));
        turretSubsystem.setLauncher(Utils.getLauncherRPMForDistance(distance));

        if(turretSubsystem.isLauncherUpToSpeed() && waitCommand.isFinished()){
            hopperSubsystem.feedKicker();
        }else{
            hopperSubsystem.stopKicker();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        waitCommand.cancel();
        hopperSubsystem.spindexerOff();
        hopperSubsystem.stopKicker();

        turretSubsystem.setLauncher(0);
        turretSubsystem.setTargetElevation(Constants.ShooterConstants.DEFLECTOR_STORED_ANGLE);

        Logger.recordOutput("AutoTurretShoot-State", "NOT SHOOTING");

        DriverStation.getAlliance().ifPresent((aliance) -> {
            if (aliance == Alliance.Blue) {
                turretSubsystem.setTurretTargetPose(Constants.FieldPoses.BLUE_HUB_POSE);
            } else {
                turretSubsystem.setTurretTargetPose(Constants.FieldPoses.RED_HUB_POSE);
            }
        });

        // turretSubsystem.setTurretMotorRotation(-0.34); // Return to 0 after MIAMI VALLEY
    }

}
