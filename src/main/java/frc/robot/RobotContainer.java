// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ManualHopperCommand;
import frc.robot.commands.ManualShootCommand;
import frc.robot.commands.SetDeflectorCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;

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

  public final DriveSubsystem driveSubsystem = new DriveSubsystem(m_driverController);

  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  public final TurretSubsystem turretSubsystem = new TurretSubsystem();

  public final HopperSubsystem hopperSubsystem = new HopperSubsystem();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

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
    turretSubsystem.setDefaultCommand(new ManualShootCommand(turretSubsystem, m_driverController.getHID()));
    hopperSubsystem.setDefaultCommand(new ManualHopperCommand(hopperSubsystem, m_driverController.getHID()));

    //The shooter will tear its self apart if deflector moves, so dont move it

    /*m_driverController.povDown().onTrue(new SetDeflectorCommand(turretSubsystem, Constants.ShooterConstants.DEFLECTOR_STORED_ANGLE));
    m_driverController.povLeft().onTrue(new SetDeflectorCommand(turretSubsystem, Constants.ShooterConstants.DEFLECTOR_SET_ANGLE1));
    m_driverController.povRight().onTrue(new SetDeflectorCommand(turretSubsystem, Constants.ShooterConstants.DEFLECTOR_SET_ANGLE3));
    m_driverController.povUp().onTrue(new SetDeflectorCommand(turretSubsystem, Constants.ShooterConstants.DEFLECTOR_SET_ANGLE2));

    m_driverController.y().onTrue(intakeSubsystem.deployCommand());
    m_driverController.a().onTrue(intakeSubsystem.retractCommand());
    m_driverController.b().onTrue(intakeSubsystem.reverseCommand());*/

    m_driverController.x().onTrue(intakeSubsystem.killSpeedCommand());
    m_driverController.a().onTrue(intakeSubsystem.startSpeedCommand());
    m_driverController.b().onTrue(intakeSubsystem.revSpeedCommand());
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
