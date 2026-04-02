package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class RelayCommand extends Command{

    private HopperSubsystem hopperSubsystem;
    private TurretSubsystem turretSubsystem;
    private IntakeSubsystem intakeSubsystem;

    private Command waitCommand = Commands.waitSeconds(1.5);

    public RelayCommand(HopperSubsystem hopperSubsystem, TurretSubsystem turretSubsystem, IntakeSubsystem intakeSubsystem){
        this.hopperSubsystem = hopperSubsystem;
        this.turretSubsystem = turretSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(hopperSubsystem, turretSubsystem, intakeSubsystem);
    }
    
    @Override
    public void initialize() {
        turretSubsystem.setTargetElevation(Constants.ShooterConstants.DEFLECTOR_SET_ANGLE3);
        turretSubsystem.setLauncher(5500);
        hopperSubsystem.spindexerOn();
        CommandScheduler.getInstance().schedule(waitCommand);
    }

    @Override
    public void execute() {
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
    }

}
