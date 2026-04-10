package frc.robot;

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

}
