package frc.robot.commands;

import java.lang.constant.Constable;

import org.fairportrobotics.frc.robolib.vision.limelight.LimelightHelpers;
import org.fairportrobotics.frc.robolib.vision.limelight.LimelightHelpers.LimelightTarget_Fiducial;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AutoShootCommandChassisTurret extends Command{
    
    // For Chassis Control
    private DriveSubsystem driveSubsystem;
    
    private HopperSubsystem hopperSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private TurretSubsystem turretSubsystem;

    private PIDController autoCenterController;

    public AutoShootCommandChassisTurret(DriveSubsystem driveSubsystem, HopperSubsystem hopperSubsystem, IntakeSubsystem intakeSubsystem, TurretSubsystem turretSubsystem)
    {
        this.driveSubsystem = driveSubsystem;
        this.hopperSubsystem = hopperSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.turretSubsystem = turretSubsystem;

        autoCenterController = new PIDController(0, 0, 0);
        autoCenterController.setTolerance(1);
        autoCenterController.setSetpoint(0);
    }

    @Override
    public void initialize() {
        turretSubsystem.setLauncher(2500);
        hopperSubsystem.spindexerOn();
    }

    @Override
    public void execute() {
        // Need to compute shoot speed and angle

        // Get apriltags
        // Change to HUB tracking pipeline
        LimelightHelpers.setPipelineIndex(Constants.CameraConstanst.BACK_CAMERA_NAME, Constants.CameraConstanst.BACK_HUB_TRACKING_PIPELINE_NUMBER);

        LimelightHelpers.getTargetPose_CameraSpace(Constants.CameraConstanst.BACK_CAMERA_NAME);

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

        LimelightHelpers.setPipelineIndex(Constants.CameraConstanst.BACK_CAMERA_NAME, Constants.CameraConstanst.BACK_LOCALIZATION_PIPELINE_NUMBER);
    }

}
