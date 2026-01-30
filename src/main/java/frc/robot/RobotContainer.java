// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.fairportrobotics.frc.robolib.drivesystems.swerve.SwerveBuilder;
import org.fairportrobotics.frc.robolib.drivesystems.swerve.SwerveDriveSubsystem;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // private final CommandPS4Controller m_driverController = new CommandPS4Controller(OperatorConstants.kDriverControllerPort);

  public static SwerveDriveSubsystem driveSubsystem;

  private double driveP = 0.1;
  private double driveI = 0.0;
  private double driveD = 0.01;
  private double driveKV = 0.2;

  private double steerP = 50;
  private double steerI = 1;
  private double steerD = 1;
  private double steerKV = 0;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    SwerveBuilder swerveBuilder = new SwerveBuilder();

    driveSubsystem = swerveBuilder
                    .withCanbusName("Drive")
                    .withPigeonId(20)
                    .withMaxLinearVelocity(3.0)
                    .withMaxAngularVelocity(Math.PI * 2)
                    .withSwerveModule(
                      swerveBuilder.new SwerveModuleBuilder()
                      .withDriveMotorId(12)
                      .withDriveKP(driveP)
                      .withDriveKI(driveI)
                      .withDriveKD(driveD)
                      .withDriveKV(driveKV)
                      .withSteerMotorId(3)
                      .withSteerKP(steerP)
                      .withSteerKI(steerI)
                      .withSteerKD(steerD)
                      .withSteerKV(steerKV)
                      .withSteerOffset(0.048828)
                      .withSteerEncoderId(10)
                      .withModuleLocation(new Translation2d(1, 1))
                      .withModuleName("Front Left")
                      .withGearRatio(8.14)
                      .withWheelDiameter(0.1016)
                      .build()
                    )
                    .withSwerveModule(
                      swerveBuilder.new SwerveModuleBuilder()
                      .withDriveMotorId(5)
                      .withDriveKP(driveP)
                      .withDriveKI(driveI)
                      .withDriveKD(driveD)
                      .withDriveKV(driveKV)
                      .withDriveInverted()
                      .withSteerMotorId(6)
                      .withSteerKP(steerP)
                      .withSteerKI(steerI)
                      .withSteerKD(steerD)
                      .withSteerKV(steerKV)
                      .withSteerOffset(0.244875)
                      .withSteerEncoderId(1)
                      .withModuleLocation(new Translation2d(1, -1))
                      .withModuleName("Front Right")
                      .withGearRatio(8.14)
                      .withWheelDiameter(0.1016)
                      .build())
                    .withSwerveModule(
                      swerveBuilder.new SwerveModuleBuilder()
                      .withDriveMotorId(8)
                      .withDriveKP(driveP)
                      .withDriveKI(driveI)
                      .withDriveKD(driveD)
                      .withDriveKV(driveKV)
                      .withSteerMotorId(2)
                      .withSteerKP(steerP)
                      .withSteerKI(steerI)
                      .withSteerKD(steerD)
                      .withSteerKV(steerKV)
                      .withSteerOffset(0.430176)
                      .withSteerEncoderId(7)
                      .withModuleLocation(new Translation2d(-1, 1))
                      .withModuleName("Back Left")
                      .withGearRatio(8.14)
                      .withWheelDiameter(0.1016)
                      .build())
                    .withSwerveModule(
                      swerveBuilder.new SwerveModuleBuilder()
                      .withDriveMotorId(9)
                      .withDriveKP(driveP)
                      .withDriveKI(driveI)
                      .withDriveKD(driveD)
                      .withDriveKV(driveKV)
                      .withDriveInverted()
                      .withSteerMotorId(11)
                      .withSteerKP(steerP)
                      .withSteerKI(steerI)
                      .withSteerKD(steerD)
                      .withSteerKV(steerKV)
                      .withSteerOffset(0.012695)
                      .withSteerEncoderId(4)
                      .withModuleLocation(new Translation2d(-1, -1))
                      .withModuleName("Back Right")
                      .withGearRatio(8.14)
                      .withWheelDiameter(0.1016)
                      .build())
                    .build();

    driveSubsystem.setDefaultCommand(getDriveCommand());

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  public Command getDriveCommand(){
    return Commands.run(new Runnable() {

      @Override
      public void run() {
        driveSubsystem.setChassisSpeedsFromJoystickRobotRelative(
          -(m_driverController.getLeftY() * Math.abs(m_driverController.getLeftY())),
          -(m_driverController.getLeftX() * Math.abs(m_driverController.getLeftX())),
          -(m_driverController.getRawAxis(4) * Math.abs(m_driverController.getRawAxis(4))));

        // driveSubsystem.setChassisSpeed(new ChassisSpeeds(1, 0, 0));

        Logger.recordOutput("RequestedSwerveState", driveSubsystem.getRequestedModuleStates());
        Logger.recordOutput("ActualSwerveState", driveSubsystem.getActualModuleStates());
        Logger.recordOutput("Pose Estimation", driveSubsystem.getRobotPose());
      }

    }, driveSubsystem);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
