package frc.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Constants.ShootingRegion;
import frc.robot.Constants.ShootingRegionDimensions;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Angle;

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
     * @param robotPose: the robot's pose
     * @param alliance: the red/blue alliance of the robot, as a sanity check
     * @return the robot's shooting region
     */
    public static ShootingRegion findRobotShootingRegion(Pose3d robotPose, Alliance alliance){
        ShootingRegion region = Constants.ShootingRegion.NON_SHOOTING_REGION;   // the default return value if robot isn't in a valid region
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();

        // Receommended: Add a sanity check for invalid robot poses.

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
        return region;
    }

    public static double getHoodAngleForDistance(double distanceInMeters){
        if(distanceInMeters >= 0){
            return 0;
        }
        return 0;
    }

    public static double getLauncherRPMForDistance(double distanceInMeters){ 
        return (667.557*distanceInMeters) + 1024.699;
    }

    /**
     * Determines the appropriate shooting target for the robot based on its alliance
     * and its current region on the field. There are 10 possible targets:
     * Scoring hub, 2x passing from Neutral Zone, 2x passing from opponent's Alliance Zone,
     * and these exist for both red and blue alliances.
     * Right/left are from driver's perspective.
     * @param region: the identifier of the robot's current field region
     * @param alliance: the red/blue alliance of the robot, as a sanity check
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
