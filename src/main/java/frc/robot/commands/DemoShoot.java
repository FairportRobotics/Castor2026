package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class DemoShoot extends Command{

    private HopperSubsystem hopperSubsystem;
    private TurretSubsystem turretSubsystem;

    private Command waitCommand = Commands.waitSeconds(1.7);

    public DemoShoot(HopperSubsystem hopperSubsystem, TurretSubsystem turretSubsystem){
        this.hopperSubsystem = hopperSubsystem;
        this.turretSubsystem = turretSubsystem;

        addRequirements(hopperSubsystem);
    }
    
    @Override
    public void initialize() {
        hopperSubsystem.spindexerOn();
        CommandScheduler.getInstance().schedule(waitCommand);
    }

    @Override
    public void execute() {
        turretSubsystem.setLauncher(2800);

        if(waitCommand.isFinished()) {hopperSubsystem.feedKicker();}

        else {hopperSubsystem.stopKicker();}
    }

    @Override
    public void end(boolean interrupted) {
        waitCommand.cancel();
        hopperSubsystem.spindexerOff();
        hopperSubsystem.stopKicker();
        turretSubsystem.setLauncher(0);
    }

}
