package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.ShootingRegion;
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
    }

    @Override
    public void execute() {

        ShootingRegion shootingRegion = Utils.findRobotShootingRegion(driveSubsystem.getBotPose(), alliance);

        // if(shootingRegion == )

        double distance = turretSubsystem.getTurretTargetPose().toPose2d().getTranslation().getDistance(driveSubsystem.getBotPose().getTranslation().toTranslation2d());
        Logger.recordOutput("AutoTurretShoot-DistanceToTarget(Meters)", distance);

        turretSubsystem.setTargetElevation(Utils.getHoodAngleForDistance(0));
        turretSubsystem.setLauncher(Utils.getLauncherRPMForDistance(0));

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
        // turretSubsystem.setTurretMotorRotation(-0.34); // Return to 0 after MIAMI VALLEY
    }

}
