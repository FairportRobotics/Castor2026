package frc.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Constants.ShootingRegion;
import frc.robot.Constants.ShootingRegionDimensions;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

import java.util.Map;

public class Utils {
    
    /**
     * Calculates the needed angle of the turret for a distance from the HUB.
     * @param distanceToTag
     * @return number of rotations of the turret, robot relative
     */
    public static double calculateTurretAngleFromCameraTagDistance(double distanceToTag){

        // Calculate turret Angle
        double cameraDistanceToTarget = distanceToTag + .6;
        double cameraToShootDistance = 0.26035;
        // a^2 + b^2 = c^2
        double shooterDistanceToTarget = Math
                .sqrt(Math.pow(cameraDistanceToTarget, 2) + Math.pow(cameraToShootDistance, 2));
        double angleAtShooter = Math.acos(cameraToShootDistance / shooterDistanceToTarget);
        double robotTurretAngle = (Math.PI / 2) - angleAtShooter;
        double robotTurretRotations = (robotTurretAngle / Math.PI);

        return robotTurretRotations;
    }



    /**
     * Determines the region for the robot based on its location and alliance.
     * Assumes the robot is a 27" square. Thresholds based on observations from simulator.
     * @param robotPose the robot's pose
     * @param alliance the red/blue alliance of the robot, as a sanity check
     * @return the robot's shooting region
     */
    public static ShootingRegion findRobotShootingRegion(Pose3d robotPose, Alliance alliance){
        ShootingRegion region = Constants.ShootingRegion.NON_SHOOTING_REGION;   // the default return value if robot isn't in a valid region
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();

        // Recommended: Add a sanity check for invalid robot poses.

        if (alliance == Alliance.Blue) {
            if (robotX < Constants.ShootingRegionDimensions.BLUE_ALLIANCE_ZONE_REGION_X_MAX) {
                region = Constants.ShootingRegion.BLUE_OWN_ALLIANCE_ZONE;
                }
            else if (robotX > Constants.ShootingRegionDimensions.NEUTRAL_ZONE_REGION_X_MIN &&
                     robotX < Constants.ShootingRegionDimensions.NEUTRAL_ZONE_REGION_X_MAX &&
                     robotY > Constants.ShootingRegionDimensions.BLUE_LEFT_REGION_Y_MIN) {
                        region = Constants.ShootingRegion.BLUE_LEFT_NEUTRAL_ZONE;
                     }
            else if (robotX > Constants.ShootingRegionDimensions.NEUTRAL_ZONE_REGION_X_MIN &&
                     robotX < Constants.ShootingRegionDimensions.NEUTRAL_ZONE_REGION_X_MAX &&
                     robotY < Constants.ShootingRegionDimensions.BLUE_RIGHT_REGION_Y_MAX) {
                        region = Constants.ShootingRegion.BLUE_RIGHT_NEUTRAL_ZONE;                
                     }
            else if (robotX > Constants.ShootingRegionDimensions.RED_ALLIANCE_ZONE_REGION_X_MIN &&
                     robotY > Constants.ShootingRegionDimensions.BLUE_LEFT_REGION_Y_MIN) {
                        region = Constants.ShootingRegion.BLUE_LEFT_OPPONENT_ALLIANCE_ZONE;
                     }
            else if (robotX > Constants.ShootingRegionDimensions.RED_ALLIANCE_ZONE_REGION_X_MIN &&
                     robotY < Constants.ShootingRegionDimensions.BLUE_RIGHT_REGION_Y_MAX) {
                        region = Constants.ShootingRegion.BLUE_RIGHT_OPPONENT_ALLIANCE_ZONE;
                     }
        }
        else if (alliance == Alliance.Red) {
            if (robotX > Constants.ShootingRegionDimensions.RED_ALLIANCE_ZONE_REGION_X_MIN) {
                region = Constants.ShootingRegion.RED_OWN_ALLIANCE_ZONE;
                }
            else if (robotX > Constants.ShootingRegionDimensions.NEUTRAL_ZONE_REGION_X_MIN &&
                     robotX < Constants.ShootingRegionDimensions.NEUTRAL_ZONE_REGION_X_MAX &&
                     robotY < Constants.ShootingRegionDimensions.RED_LEFT_REGION_Y_MAX) {
                        region = Constants.ShootingRegion.RED_LEFT_NEUTRAL_ZONE;
                     }
            else if (robotX > Constants.ShootingRegionDimensions.NEUTRAL_ZONE_REGION_X_MIN &&
                     robotX < Constants.ShootingRegionDimensions.NEUTRAL_ZONE_REGION_X_MAX &&
                     robotY > Constants.ShootingRegionDimensions.RED_RIGHT_REGION_Y_MIN) {
                        region = Constants.ShootingRegion.RED_RIGHT_NEUTRAL_ZONE;
                     }
            else if (robotX < Constants.ShootingRegionDimensions.BLUE_ALLIANCE_ZONE_REGION_X_MAX &&
                     robotY < Constants.ShootingRegionDimensions.RED_LEFT_REGION_Y_MAX) {
                        region = Constants.ShootingRegion.RED_LEFT_OPPONENT_ALLIANCE_ZONE;
                     }
            else if (robotX > Constants.ShootingRegionDimensions.RED_ALLIANCE_ZONE_REGION_X_MIN &&
                     robotY > Constants.ShootingRegionDimensions.RED_RIGHT_REGION_Y_MIN) {
                        region = Constants.ShootingRegion.RED_RIGHT_OPPONENT_ALLIANCE_ZONE;
                     }
        }

        Logger.recordOutput("Utils-ShootingRegion", region);

        return region;
    }


    /**
     * Calculates the ideal hood angle based on the provided distance
     * between the robot's turret and the target (hub).
     * Implements the equation to find hood angle for scoring:
     * Angle = MaxAngle - ((Distance - ShortDistance) / Div), where
     *      MaxAngle = maximum hood angle for scoring
     *      Distance = distance to target
     *      ShortDistance = shortest distance used when evaluating hood angles
     *      LongDistance = longest distance used when evaluating hood angles
     *      Div = the distance from the target that requires 1° of hood angle change
     *          = (LongDistance - ShortDistance) / (MaxAngle - MinAngle)
     * These were all defined as constants in Constants.java.
     * @param distanceInMeters distance between turret and target, in meters
     * @return the ideal nearest-integer hood angle in degrees relative to the floor
     */
    public static int getScoringHoodAngleDegForDistance(double distanceInMeters){
        double calculatedAngleDeg = Constants.ShooterConstants.HOOD_ANGLE_PASSING_DEG;      // init to the passing value
        double distanceInInches = Units.metersToInches(distanceInMeters);                   // Convert to inches to match theory equations
 
        // Limit the distance according to the min and max that were used in the theory calculations
        MathUtil.clamp(distanceInInches, Constants.ShooterConstants.SCORING_DISTANCE_SHORT_IN, Constants.ShooterConstants.SCORING_DISTANCE_LONG_IN);

        // Perfom the main calculation (Equation 2-5 in the AutoShooter document), then round the result to the nearest degree
        calculatedAngleDeg = Constants.ShooterConstants.HOOD_ANGLE_SCORING_MAX_DEG - (((distanceInInches) - Constants.ShooterConstants.SCORING_DISTANCE_SHORT_IN) / 
                                    Constants.ShooterConstants.HOOD_ANGLE_CALC_DIVISION);
        int hoodAngleDeg = (int) Math.round(calculatedAngleDeg);

        return (hoodAngleDeg);
    }



    public static double getHoodAngleForDistance(double distanceInMeters){
        if(distanceInMeters >= 4){
            return 200;
        }
        return 0;
    }


    /**
     * Calculates the appropriate launch speed in ft/s for scoring based on
     * the provided hood angle and target distance in meters.
     * This is done based on a series of quadratic regressions
     * See the AutoShooter theory document.
     * @param hoodAngleDeg the hood angle that the calculation should assume
     * @param distanceInMeters distance between turret and target, in meters
     * @return the appropriate launch speed for scoring in feet per second
     * Returns 0 if a launch speed cannot be determined.
     */
    public static double findLaunchSpeedScoring(int hoodAngleDeg, double distanceInMeters){
        // Check that the requested hood angle is in the expected shooting range
        if ((hoodAngleDeg < Constants.ShooterConstants.HOOD_ANGLE_SCORING_MIN_DEG) ||
           (hoodAngleDeg > Constants.ShooterConstants.HOOD_ANGLE_SCORING_MAX_DEG)) {
            return 0;
        }

        // Check that the requested distance is reasonable. If the scoring distance is outside of this range, there was a problem.
        if ((distanceInMeters < Constants.ShooterConstants.SCORING_DISTANCE_FIELD_MIN_METERS) ||
           (distanceInMeters > Constants.ShooterConstants.SCORING_DISTANCE_FIELD_MAX_METERS)) {
            return 0;
        }
                
        // Get the quadratic coefficients for the hood angle. Return with the error value if none are found.
        Constants.ShooterConstants.QuadraticCoef coefficients = Constants.ShooterConstants.SCORING_SPEED_COEFS_METERS.get(hoodAngleDeg);
        if (coefficients == null) {
            return 0;
        }
        
        // Perform the quadratic calculation using the coefficients we retrieved if successful
        double x = distanceInMeters;                            // Rename to "x" just to make the next expression simpler
        double launchSpeedFPS = (coefficients.a() * x * x) + (coefficients.b() * x) + coefficients.c();

        return launchSpeedFPS;
    }

    /**
     * Calculates the appropriate launch speed in ft/s for passing based on
     * the provided hood angle and target distance in meters.
     * This is done based on a series of quadratic regressions
     * See the AutoShooter theory document.
     * @param hoodAngleDeg the hood angle that the calculation should assume
     * @param distanceInMeters distance between turret and target, in meters
     * @return the appropriate launch speed for passing in feet per second
     * Returns 0 if a launch speed cannot be determined.
     */
    public static double findLaunchSpeedPassing(int hoodAngleDeg, double distanceInMeters){
        // Check that the requested hood angle is in the expected angle range for passing.
        if (hoodAngleDeg != Constants.ShooterConstants.HOOD_ANGLE_PASSING_DEG) {
            return 0;
        }

        // Check that the distance is reasonable. If the scoring distance is outside of this range, there was a problem.
        if ((distanceInMeters < Constants.ShooterConstants.SCORING_DISTANCE_FIELD_MIN_METERS) ||
           (distanceInMeters > Constants.ShooterConstants.SCORING_DISTANCE_FIELD_MAX_METERS)) {
            return 0;
        }
                
        // Get the quadratic coefficients for the hood angle. Return with the error value if none are found.
        Constants.ShooterConstants.QuadraticCoef coefficients = Constants.ShooterConstants.PASSING_SPEED_COEFS_METERS.get(hoodAngleDeg);
        if (coefficients == null) {
            return 0;
        }
        
        // Perform the quadratic calculation using the coefficients we retrieved if successful
        double x = distanceInMeters;                            // Rename to "x" just to make the next expression simpler
        double launchSpeedFPS = (coefficients.a() * x * x) + (coefficients.b() * x) + coefficients.c();

        return launchSpeedFPS;
    }

    /**
     * Estimates the fuel's time of flight until it his the target,
     * where the target is the top of the hub when scoring and the floor when passing
     * launch speed in feet/second.
     * This performs Equations 3-7 and 3-8 in the AutoShooter document.
     * @param scoring true if the bot is scoring into the hub (false means passing)
     * @param launchAngleDeg the target launch angle in degrees
     * @param launchSpeedFPS the target launch speed in ft/s
     * @return the approxmate fuel flight time in seconds
     */
    public static double findTimeOfFlightScoring(boolean scoring, int launchAngleDeg, double launchSpeedFPS){
        double velocityVertFPS = launchSpeedFPS * Math.sin(Math.toRadians((double)launchAngleDeg)); // The vertical component of fuel's velocity
        double kGravityInFeet = 32.2;                                                               // Gravity constant in ft/s^2
        double deltaHFeet = 0;                                                                      // Heigh difference between launcher and target
        if (scoring) {
            deltaHFeet = (Constants.ShooterConstants.SHOOTER_HEIGHT_INCHES - Constants.ShooterConstants.HUB_OPENING_HEIGHT_INCHES) / 12;
        } else {
            deltaHFeet = Constants.ShooterConstants.SHOOTER_HEIGHT_INCHES / 12;
        }

        return ((velocityVertFPS + Math.sqrt(Math.pow(velocityVertFPS, 2) - (2 * kGravityInFeet * deltaHFeet))) / kGravityInFeet);

    }

    /**
     * Calculates the flywheel RPM that's required to achieve the provided
     * launch speed in feet/second.
     * This performs Equation 2-3 in the AutoShooter document.
     * @param launchSpeedFPS the goal launch speed in ft/s
     * @return the appropriate flywheel speed in RPM
     */
    public static double GetShooterRPMForLaunchSpeed(double launchSpeedFPS){
        double flywheelDiameterFeet = Constants.ShooterConstants.SHOOTER_FLYWHEEL_DIAMETER_IN / 12;         // Change to feet to match launch speed units
        double efficiency = Constants.ShooterConstants.SHOOTER_EFFICIENCY_GENERAL;                          // Use the appropriate efficiency
        
        double shooterRPM = (launchSpeedFPS / (efficiency * flywheelDiameterFeet * Math.PI)) * 60;          // Perform the calculation
        return shooterRPM;
    }

    public static double getLauncherRPMForDistance(double distanceInMeters){ 
        return Math.min((667.557*distanceInMeters) + 1500.699, 5000);
    }

    /**
     * Determines the appropriate shooting target for the robot based on its alliance
     * and its current region on the field. There are 10 possible targets:
     * Scoring hub, 2x passing from Neutral Zone, 2x passing from opponent's Alliance Zone,
     * and these exist for both red and blue alliances.
     * Right/left are from driver's perspective.
     * @param region the identifier of the robot's current field region
     * @param alliance the red/blue alliance of the robot, as a sanity check
     * @return the shooting target for a robot with the provided properties. Returns all-zero
     * Pose3d object if the region/alliance combination is invalid.
     */
    public static Pose3d findShootingTarget(ShootingRegion region, Alliance alliance){

        Pose3d shootingTarget = new Pose3d();
        if (alliance == Alliance.Blue) {
            switch (region) {
                case BLUE_OWN_ALLIANCE_ZONE:
                    shootingTarget = Constants.FieldPoses.BLUE_HUB_POSE;
                    break;
                case BLUE_LEFT_NEUTRAL_ZONE:
                    shootingTarget = Constants.FieldPoses.BLUE_LEFT_ALLIANCE_ZONE_PASS_TARGET_POSE;
                    break;
                case BLUE_RIGHT_NEUTRAL_ZONE:
                    shootingTarget = Constants.FieldPoses.BLUE_RIGHT_ALLIANCE_ZONE_PASS_TARGET_POSE;
                    break;
                case BLUE_LEFT_OPPONENT_ALLIANCE_ZONE:
                    shootingTarget = Constants.FieldPoses.BLUE_LEFT_NEUTRAL_ZONE_PASS_TARGET_POSE;
                    break;
                case BLUE_RIGHT_OPPONENT_ALLIANCE_ZONE:
                    shootingTarget = Constants.FieldPoses.BLUE_RIGHT_NEUTRAL_ZONE_PASS_TARGET_POSE;
                    break;
                default:
                    break;
            }
        }
        else if (alliance == Alliance.Red) {
            switch (region) {
                case RED_OWN_ALLIANCE_ZONE:
                    shootingTarget = Constants.FieldPoses.RED_HUB_POSE;
                    break;
                case RED_LEFT_NEUTRAL_ZONE:
                    shootingTarget = Constants.FieldPoses.RED_LEFT_ALLIANCE_ZONE_PASS_TARGET_POSE;
                    break;
                case RED_RIGHT_NEUTRAL_ZONE:
                    shootingTarget = Constants.FieldPoses.RED_RIGHT_ALLIANCE_ZONE_PASS_TARGET_POSE;
                    break;
                case RED_LEFT_OPPONENT_ALLIANCE_ZONE:
                    shootingTarget = Constants.FieldPoses.RED_LEFT_NEUTRAL_ZONE_PASS_TARGET_POSE;
                    break;
                case RED_RIGHT_OPPONENT_ALLIANCE_ZONE:
                    shootingTarget = Constants.FieldPoses.RED_RIGHT_NEUTRAL_ZONE_PASS_TARGET_POSE;
                    break;
                default:
                    break;
            }
        }
        return shootingTarget;
    }

}
