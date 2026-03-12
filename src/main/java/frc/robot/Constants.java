// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import java.security.PublicKey;

import com.ctre.phoenix6.signals.InvertedValue;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class AdvantageKitConstants{
    public enum RobotType{
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
   * Single digit numbers (E.g. 1, 2, 3) are extra, non-motor, pieces that require IDs, such as the Pigeon
   * Numbers starting with 1 (E.g. 11, 12, 13) are in the Front Left of the robot
   * Numbers starting with 2 (E.g. 21, 22, 23) are in the Front Right of the robot
   * Numbers starting with 3 (E.g. 31, 32, 33) are in the Back Left of the robot
   * Numbers starting with 4 (E.g. 41, 42, 43) are in the Back Right of the robot
   * Numbers starting with 5 (E.g. 51, 52, 53) are on the Intake
   * Numbers starting with 6 (E.g. 61, 62, 63) are on the Shooter
   * Numbers starting with 7 (E.g. 71, 72, 73) are on the Hopper
   * Later numbers will be added for more subsystems
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
  
  public static class ExtraIDEntities {
    public static final int PIGEON_ID = 1;
    //If the Limelights have IDs, add them here.
  }
  
  public static class IntakeConstants {
    public static final int DEPLOY_MOTOR_ID = 51;
    public static final int INTAKE_MOTOR_ID = 52;
  }

  public static class ShooterConstants {
    public static final int LIMIT_CHANNEL = 0;
    public static final int TURRET_ID = 61;
    public static final int LAUNCHER_ID = 62;
    public static final int HOOD_CHANNEL = 3;
    public static final int HOOD_ENCODER_ID = 62;
    public static final Angle LIMIT_AXIMUTH_POS = edu.wpi.first.units.Units.Degrees.of(265);
    public static final Angle LIMIT_AXIMUTH_NEG = edu.wpi.first.units.Units.Degrees.of(5);
    public static final double HOMING_SPEED = .35;
    public static final double TURRET_GEAR_RATIO  = 1;

    public static final int HOOD_SERVO_CHANNEL = 3;
    public static final boolean HOOD_SERVO_INVERTED = true; // TODO: Test this
    public static final int HOOD_ENCODER_ID = 4;
    public static final Angle TARGET_ELEVATION_DIF = edu.wpi.first.units.Units.Degrees.of(1);
    public static final Angle LIMIT_ELEVATION_POS = edu.wpi.first.units.Units.Degrees.of(90);
    public static final Angle LIMIT_ELEVATION_NEG = edu.wpi.first.units.Units.Degrees.of(0);
    public static final Angle DEFLECTOR_STORED_ANGLE = edu.wpi.first.units.Units.Degrees.of(0);
    public static final Angle DEFLECTOR_SET_ANGLE1 = edu.wpi.first.units.Units.Degrees.of(20);
    public static final Angle DEFLECTOR_SET_ANGLE2 = edu.wpi.first.units.Units.Degrees.of(40);
    public static final Angle DEFLECTOR_SET_ANGLE3 = edu.wpi.first.units.Units.Degrees.of(60);
  }
  /*
   * Making comments is fun AND helpful if they actually tell you something about the code.
   * This one doesn't.
   */
  public static class HopperConstants {
    public static final int KICKER_MOTOR_ID = 71;
    public static final int SPINDEXER_MOTOR_ID = 72;
  }
}
