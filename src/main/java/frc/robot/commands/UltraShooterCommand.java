package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import frc.robot.subsystems.TurretSubsystem;


public class UltraShooterCommand extends Command{

    private HopperSubsystem hopperSubsystem;
    private TurretSubsystem turretSubsystem;
    private DriveSubsystem driveSubsystem;

    private Command waitCommand = Commands.waitSeconds(1.5);
    private final Translation2d CENTER_TURRET_TO_ROBOT = new Translation2d(-0.1335024, -0.1824736);     // Defines physical offset of turret from robot center

    private Alliance alliance;                                              // Shooting behavior is based on the robot's alliance
    
    public UltraShooterCommand(HopperSubsystem hopperSubsystem, TurretSubsystem turretSubsystem, DriveSubsystem driveSubsystem){
        this.hopperSubsystem = hopperSubsystem;
        this.turretSubsystem = turretSubsystem;
        this.driveSubsystem = driveSubsystem;

        addRequirements(hopperSubsystem);
    }
    
    @Override
    public void initialize() {
        hopperSubsystem.spindexerOn();                                      // Activate spindexer so it's on while command is active
        hopperSubsystem.stopKicker();                                       // Make sure kicker is off when command is called
        CommandScheduler.getInstance().schedule(waitCommand);               // Kick off a wait timer so robot doesn't shoot until hopper is ready
        alliance = DriverStation.getAlliance().get();                       // Fetch our alliance
        Logger.recordOutput("UltraShooter-State", "ACTIVE");
    }

    @Override
    public void execute() {
        Boolean okToShoot = false;                      // Tracks whether conditions are OK for shooting.
        Boolean scoring = false;                        // Are we shooting into the hub?
        Pose3d botPose = driveSubsystem.getBotPose();   // Get and store the bot's pose once since we'll use it a few times
        
        // STEPS 1 & 2: Get the field location of the initial target
        // First, find the robot's current field location.
        ShootingRegion shootingRegion = Utils.findRobotShootingRegion(botPose, alliance);

        // If robot is in a valid scoring or passing region, find the field pose for the appropriate target and perform calculations
        if(shootingRegion != ShootingRegion.NON_SHOOTING_REGION){
            
            okToShoot = true;                                                                   // So far, so good
            
            // Check whether we're scoring, since it require some extra steps
            if ((shootingRegion == ShootingRegion.BLUE_OWN_ALLIANCE_ZONE) || (shootingRegion == ShootingRegion.RED_OWN_ALLIANCE_ZONE)){
                scoring = true;
            }
                        
            Pose3d initialTargetPose = Utils.findShootingTarget(shootingRegion, alliance);      // Get initial target based on robot location
            Logger.recordOutput("UltraShooter-InitialTarget", initialTargetPose);

            // STEPS 3 & 4: calculate hood and flywheel values based on distance to the intiial target.
            // First, find the turret's pose based on the robot's pose and the location of the turret relative to the robot's center.
            // Note that the turret points backward, so its rotational property is +0.5 rotations.
            Pose3d turretPose = botPose.plus(new Transform3d(new Translation3d(CENTER_TURRET_TO_ROBOT),
                    new Rotation3d(Rotation2d.fromRotations(turretSubsystem.getTurretAngleRobotRelative() + 0.5))));
            // Then calculate the linear distance between the initial target and the robot's turret.
            double distanceToInitialTargetMeters = initialTargetPose.toPose2d().getTranslation().getDistance(turretPose.getTranslation().toTranslation2d());
            Logger.recordOutput("UltraShooter-DistanceToInitialTarget(Meters)", distanceToInitialTargetMeters);

            // STEP 3: Find the best hood angle
            // Default to the passing hood angle if we're not shooting into a hub.
            int initialHoodAngleDegrees = Constants.ShooterConstants.HOOD_ANGLE_PASSING_DEG;
            // If we're scoring, run the calculations to find the best hood angle.
            if (scoring) {
                initialHoodAngleDegrees = Utils.getScoringHoodAngleDegForDistance(distanceToInitialTargetMeters);
            }

            // STEP 4: determine the right shooter flywheel speed
            // First, find the correct launch speed in feet/second. Note the unit changes!
            // This covers the scoring scenario then the passing scenario.
            double intitialLaunchSpeedFPS = 0;                                                  // The target launch speed in feet per second (FPS)
            if (scoring) {
                intitialLaunchSpeedFPS = Utils.findLaunchSpeedScoring(initialHoodAngleDegrees, distanceToInitialTargetMeters);
            } else {
                intitialLaunchSpeedFPS = Utils.findLaunchSpeedPassing(initialHoodAngleDegrees, distanceToInitialTargetMeters);
            }

            // Handle error condition in the launch speed calcs
            if (intitialLaunchSpeedFPS == 0) {
                okToShoot = false;                
            }

            // STEP 5: Find final target based on the intial target and the robot's velocity.
            // VELOCITY ADJUSTMENTS ARE COMMENTED OUT, SO ASSUME STATIONARY ROBOT. Make final target the same as initial.
            Pose3d finalTargetPose = initialTargetPose;
            double distanceToFinalTargetMeters = distanceToInitialTargetMeters;
            Logger.recordOutput("UltraShooter-FinalTarget", finalTargetPose);
            Logger.recordOutput("UltraShooter-DistanceToFinalTarget(Meters)", distanceToFinalTargetMeters);

            /*
            // VELOCITY ADJUSTEMENTS - READY TO TRY. Uncomment this block.
            // Estimate the fuel's time of flight.
            double timeOfFlightSeconds = Utils.findTimeOfFlightScoring(scoring, initialHoodAngleDegrees, intitialLaunchSpeedFPS);
            // TBA: Get the bot's current velocity
                // SwerveDriveSystem in RoboLib must make its ChassisSpeeds accessible.
                // Get the robot-relative ChassisSpeeds and robot heading from the swerve system.
                // Then create field-relative ChassisSpeeds using the robot heading.
            
            ChassisSpeeds robotBotRelativeSpeeds = driveSubsystem.driveSystem.GetRobotRelativeSpeeds();
            Rotation2d botHeading = driveSubsystem.driveSystem.GetRobotHeading();   // TODO: adjust based on alliance since this comes straight from the Gyro?
            ChassisSpeeds robotFieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotBotRelativeSpeeds, botHeading);
            
            // Get X and Y velocity values.
            double botXVelMPS = robotFieldRelativeSpeeds.vxMetersPerSecond();
            double botYVelMPS = robotFieldRelativeSpeeds.vyMetersPerSecond();
            
            // Get translaion of initialTarget.
            Translation3d initialTargetTr = initialTargetPose.getTranslation();
            
            // Find X and Y displacements during flight.
            double targetXDisplacementMeters = botXVelMPS * timeOfFlightSeconds;
            double targetYDisplacementMeters = botYVelMPS * timeOfFlightSeconds;

            // Find final target translation.
            Translation3d adjustedTarget = new Translation3d(initialTargetTr.getX() - targetXDisplacementMeters,
	                                                         initialTargetTr.getY() - targetYDisplacementMeters,
	                                                         initialTargetTr.getZ());					// We don’t need to adjust z.

            // TBA: Generate a Pose3d for the finalTarget
            finalTargetPose = new Pose3d(adjustedTarget, initialTargetPose.getRotation());
            */

            // STEP 6: Find hood angle and launch speed for the final target.
            // THIS IS NOT IMPLEMENTED - SO ASSUME STATIONARY ROBOT. Make final target the same as initial.
            double finalLaunchSpeedFPS = intitialLaunchSpeedFPS;
            int finalHoodAngleDegrees = initialHoodAngleDegrees;
            Logger.recordOutput("UltraShooter-FinalLaunchSpeedFPS", finalLaunchSpeedFPS);
            Logger.recordOutput("UltraShooter-FinalLaunchAngleDeg", finalHoodAngleDegrees);
            
            // Confirm we're at a valid scoring distance for the robot's capabilities. If not, don't shoot.
            if ((scoring) && 
                ((distanceToFinalTargetMeters > Constants.ShooterConstants.SCORING_DISTANCE_MAX_ALLOWED_METERS) ||
                (distanceToFinalTargetMeters < Constants.ShooterConstants.SCORING_DISTANCE_MIN_ALLOWED_METERS))) {
                    okToShoot = false;
            }

            // STEP 7: Find the flywheel RPM needed to achieve the launch speed we just calculated.
            double finalShooterRPM = Utils.GetShooterRPMForLaunchSpeed(finalLaunchSpeedFPS);
            Logger.recordOutput("UltraShooter-FinalShooterFlywheelRPM", finalShooterRPM);

            // STEP 8: Command turretSubsystem to do what we need it to do for the launch speed and launch angle.
            // Adjust for the shooter flywheel-to-motor gear ratio when setting the flywheel motor speed.
            turretSubsystem.setHoodToLaunchAngle(finalHoodAngleDegrees);
            turretSubsystem.setLauncher(finalShooterRPM / Constants.ShooterConstants.SHOOTER_FLYWHEEL_GEAR_RATIO);

            // STEP 9: Command the turretSubsystem to aim at the intended field target (azimuth control).
            // First, find the x,y translation between the field positions of the robot's turret and the final target.
            Translation2d finalTargetTranslation = finalTargetPose.getTranslation().toTranslation2d().minus(turretPose.getTranslation().toTranslation2d());
            
            // Find the rotations value, field-relative, between turret point and target point.
            double turretToTargetRotation = finalTargetTranslation.getAngle().getRotations();
            
            // Command the turret to get to this field-relative angle.
            // Don't try to set the turret if it's outside of it's physical capability, and prevent shooting.
            if(turretSubsystem.isTurretReady() && turretSubsystem.isTurretAnglePossible(botPose, finalTargetTranslation)) {
                turretSubsystem.setTurretFieldRelative(botPose, turretToTargetRotation);
            } else {
                okToShoot = false;
            }

        } else {
            // If we're in a non-shooting region while the shooting function is active,
            // set the hood to a low setting (high angle) to get under the trench and prevent shooting.
            turretSubsystem.setHoodToLaunchAngle(Constants.ShooterConstants.HOOD_ANGLE_MAX_DEG);
            okToShoot = false;
        }
        
        // STEP 10: Run the kicker if all the calculations went ok, the turret is pointed at the target, and the motors are ready
        // Note we don't call turretSubsystem.isLauncherUpToSpeed() because it currently does nothing.
        if (waitCommand.isFinished() && (okToShoot)) {
            hopperSubsystem.feedKicker();
        } else{
            hopperSubsystem.stopKicker();
        }
        Logger.recordOutput("UltraShooter-OkToShoot", okToShoot);

        // OPTIONAL: Activate rumble if it's not ok to shoot
        if (!okToShoot) {
            // ACTIVATE CONTROLLER RUMBLE
        } else {
            // DEACTIVATE CONTROLLER RUMBLE
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

        Logger.recordOutput("UltraShooter-State", "INACTIVE");

        // Removed the following since we're don't setting turret targets.
        /*
        DriverStation.getAlliance().ifPresent((alliance) -> {
            if (alliance == Alliance.Blue) {
                turretSubsystem.setTurretTargetPose(Constants.FieldPoses.BLUE_HUB_POSE);
            } else {
                turretSubsystem.setTurretTargetPose(Constants.FieldPoses.RED_HUB_POSE);
            }
        });
        */

        // turretSubsystem.setTurretMotorRotation(-0.34); // Return to 0 after MIAMI VALLEY
        // TODO: Deactivate rumble if we decided to use the rumble feature when the robot prevents shooting
    }

}
