// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class OperatorConstants {

        public static final int kDriverControllerPort = 0;
    }

    public static class CameraConstanst {

        public static class IDFilters {

            public static final int[] RED_HUB_SHOOTING_IDS = new int[]{8, 9, 10, 11};
            public static final int[] BLUE_HUB_SHOOTING_IDS = new int[]{27, 26, 25, 24};
        }

        public static final String BACK_CAMERA_NAME = "limelight-back";
        public static final int BACK_LOCALIZATION_PIPELINE_NUMBER = 0;
        public static final int BACK_HUB_TRACKING_2D_PIPELINE_NUMBER = 1;
        public static final int BACK_HUB_TRACKING_3D_PIPELINE_NUMBER = 2;

        public static final String FRONT_CAMERA_NAME = "limelight-front";
        public static final int FRONT_LOCALIZATION_PIPELINE_NUMBER = 0;
        public static final double[] BLUE_ZONE_COOR = new double[]{0, 3.5};
        public static final double[] RED_ZONE_COOR = new double[]{13, 16};
        public static final double[] NEUTRAL_ZONE_COOR = new double[]{5.5, 11};
        public static final double[] NEUTRAL1_ZONE_COOR = new double[]{5.5, 11, 0, 3};
        public static final double[] NEUTRAL2_ZONE_COOR = new double[]{5.5, 11, 5, 8};
        public static final double[] BLUE1_ZONE_COOR = new double[]{0, 3.5, 0, 3};
        public static final double[] BLUE2_ZONE_COOR = new double[]{0, 3.5, 5, 8};
        public static final double[] RED1_ZONE_COOR = new double[]{13, 16, 0, 3};
        public static final double[] RED2_ZONE_COOR = new double[]{13, 16, 5, 8};

    }

    public static class AdvantageKitConstants {

        public enum RobotType {
            SIM,
            REAL,
            REPLAY
        }
        public static final RobotType CURRENT_MODE = RobotType.REAL;
    }
    // Swerve offsets as of 3/2/26

    public static class SwerveDriveOffsets {

        public static final float FRONT_LEFT_MOTOR_OFFSET = -0.336426f;
        public static final float FRONT_RIGHT_MOTOR_OFFSET = 0.108887f;
        public static final float BACK_LEFT_MOTOR_OFFSET = -0.492432f;
        public static final float BACK_RIGHT_MOTOR_OFFSET = 0.470215f;
    }

    /*
   * Numbers starting with 1 (E.g. 11, 12, 13) are in the Front Left of the robot
   * Numbers starting with 2 (E.g. 21, 22, 23) are in the Front Right of the robot
   * Numbers starting with 3 (E.g. 31, 32, 33) are in the Back Left of the robot
   * Numbers starting with 4 (E.g. 41, 42, 43) are in the Back Right of the robot
   * Laters numbers will be added for more subsystems
   * Single diget numbers (E.g. 1, 2, 3) are extra, non-motor, pieces that require IDs, such as the Pigeon
     */
    public static class SwerveDriveIDs {

        public static final int FRONT_LEFT_STEER_ID = 11;
        public static final int FRONT_LEFT_DRIVE_ID = 12;
        public static final int FRONT_LEFT_ENCODER_ID = 13;
        public static final int FRONT_RIGHT_STEER_ID = 21;
        public static final int FRONT_RIGHT_DRIVE_ID = 22;
        public static final int FRONT_RIGHT_ENCODER_ID = 23;
        public static final int BACK_LEFT_STEER_ID = 31;
        public static final int BACK_LEFT_DRIVE_ID = 32;
        public static final int BACK_LEFT_ENCODER_ID = 33;
        public static final int BACK_RIGHT_STEER_ID = 41;
        public static final int BACK_RIGHT_DRIVE_ID = 42;
        public static final int BACK_RIGHT_ENCODER_ID = 43;
    }

    public static class RobotChassisLimits {
        // Max velocity in meters per second

        public static final double MAX_ROBOT_LINEAR_VELOCITY = 3;
        // In meters per second squared
        public static final double MAX_ROBOT_LINEAR_VELOCITY_ACCEL = 2;

        // In radians per second
        public static final double MAX_ROBOT_ANGULAR_VELOCITY = Math.PI;
        // In radians per second squared
        public static final double MAX_ROBOT_ANGULAR_VELOCITY_ACCEL = Math.PI;
    }

    //Bird
    public static class ExtraIDEntities {

        public static final int PIGEON_ID = 1;
    }

    public static class IntakeConstants {

        public static final int DEPLOY_MOTOR_ID = 17;
        public static final boolean DEPLOY_MOTOR_INVERTED = true; // TODO: Test this

        public static final int INTAKE_MOTOR_ID = 2;
        public static final InvertedValue INTAKE_MOTOR_DIRECTION = InvertedValue.Clockwise_Positive; // TODO: Test this
    }

    public static class ShooterConstants {

        public static final int LAUNCHER_MOTOR_ID = 3;
        public static final boolean LAUNCHER_MOTOR_INVERTED = false;

        public static final int TURRET_MOTOR_ID = 4;
        public static final InvertedValue TURRET_MOTOR_DIRECTION = InvertedValue.CounterClockwise_Positive;
        public static final int TURRET_LIMIT_CHANNEL = 9;
        public static final Angle LIMIT_AXIMUTH_POS = edu.wpi.first.units.Units.Degrees.of(165);
        public static final Angle LIMIT_AXIMUTH_NEG = edu.wpi.first.units.Units.Degrees.of(5);
        public static final double HOMING_SPEED = .05;
        public static final double TURRET_GEAR_RATIO = 6;

        public static final int HOOD_SERVO_CHANNEL = 0;
        public static final boolean HOOD_SERVO_INVERTED = false; // TODO: Test this
        public static final int HOOD_ENCODER_ID = 4;
        public static final Angle TARGET_ELEVATION_DIF = edu.wpi.first.units.Units.Degrees.of(1);
        public static final Angle LIMIT_ELEVATION_POS = edu.wpi.first.units.Units.Degrees.of(90);
        public static final Angle LIMIT_ELEVATION_NEG = edu.wpi.first.units.Units.Degrees.of(0);
        public static final double DEFLECTOR_STORED_ANGLE = 0;
        public static final double DEFLECTOR_SET_ANGLE1 = 100;
        public static final double DEFLECTOR_SET_ANGLE2 = 200;
        public static final double DEFLECTOR_SET_ANGLE3 = 300;
        public static final double DEFLECTOR_SERVO_RATIO = 1.0;
    }

    public static class HopperConstants {

        public static final int KICKER_MOTOR_ID = 9;
        public static final InvertedValue KICKER_MOTOR_DIRECTION = InvertedValue.Clockwise_Positive;

        public static final int SPINDEXER_MOTOR_ID = 8;
        public static final InvertedValue SPINDEXER_MOTOR_DIRECTION = InvertedValue.CounterClockwise_Positive;
    }

    public static class FieldPoses {

        private static AprilTagFieldLayout fieldTags = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        public static Pose3d BLUE_HUB_POSE = fieldTags.getTagPose(26).get().transformBy(new Transform3d(new Transform2d(-0.5207, 0, Rotation2d.kZero)));
        public static Pose3d RED_HUB_POSE = fieldTags.getTagPose(10).get().transformBy(new Transform3d(new Transform2d(-0.5207, 0, Rotation2d.kZero)));

        // These are the field targets for passing fuel to the Alliance Zone or to the Neutral Zone from the opponent's side.
        // Left/Right are from driver's perspective. Coordinates assume origin at blue side of field.
        // Rotation are zero because they don't matter.
        public static Pose3d BLUE_LEFT_ALLIANCE_ZONE_PASS_TARGET_POSE = new Pose3d(1.99, 6.35, 0, new Rotation3d());
        public static Pose3d BLUE_RIGHT_ALLIANCE_ZONE_PASS_TARGET_POSE = new Pose3d(1.99, 1.72, 0, new Rotation3d());
        public static Pose3d BLUE_LEFT_NEUTRAL_ZONE_PASS_TARGET_POSE = new Pose3d(8.27, 6.35, 0, new Rotation3d());
        public static Pose3d BLUE_RIGHT_NEUTRAL_ZONE_PASS_TARGET_POSE = new Pose3d(8.27, 1.72, 0, new Rotation3d());

        public static Pose3d RED_LEFT_ALLIANCE_ZONE_PASS_TARGET_POSE = new Pose3d(14.55, 1.72, 0, new Rotation3d());
        public static Pose3d RED_RIGHT_ALLIANCE_ZONE_PASS_TARGET_POSE = new Pose3d(14.55, 6.35, 0, new Rotation3d());
        public static Pose3d RED_LEFT_NEUTRAL_ZONE_PASS_TARGET_POSE = new Pose3d(8.27, 1.72, 0, new Rotation3d());
        public static Pose3d RED_RIGHT_NEUTRAL_ZONE_PASS_TARGET_POSE = new Pose3d(8.27, 6.35, 0, new Rotation3d());


    }

    // These represent the dividing lines for the regions in our shooting strategy.
    public static class ShootingRegionDimensions {

        public static double BLUE_ALLIANCE_ZONE_REGION_X_MAX = 3.5;
        public static double RED_ALLIANCE_ZONE_REGION_X_MIN = 13.0;
        public static double NEUTRAL_ZONE_REGION_X_MIN = 5.5;
        public static double NEUTRAL_ZONE_REGION_X_MAX = 11.0;
        public static double BLUE_RIGHT_REGION_Y_MAX = 3.0;
        public static double RED_LEFT_REGION_Y_MAX = 3.0;               // same meaning as the previous one, but from red perspective
        public static double BLUE_LEFT_REGION_Y_MIN = 5.0;
        public static double RED_RIGHT_REGION_Y_MIN = 5.0;              // same meaning as the previous one, but from red perspective

    }

    // Regions that define the AutoTurret shooting behaviors.
    // Color ID in the name reflects robot's alliance since region definitions vary by alliance.
    // Left/Right are from driver's perspective.
    public enum ShootingRegion {
        NON_SHOOTING_REGION,                // Any part of the field that's not in one of the regions
        BLUE_OWN_ALLIANCE_ZONE,
        BLUE_LEFT_NEUTRAL_ZONE,
        BLUE_RIGHT_NEUTRAL_ZONE,
        BLUE_LEFT_OPPONENT_ALLIANCE_ZONE,
        BLUE_RIGHT_OPPONENT_ALLIANCE_ZONE,
        RED_OWN_ALLIANCE_ZONE,
        RED_LEFT_NEUTRAL_ZONE,
        RED_RIGHT_NEUTRAL_ZONE,
        RED_LEFT_OPPONENT_ALLIANCE_ZONE,
        RED_RIGHT_OPPONENT_ALLIANCE_ZONE        
    }
}
