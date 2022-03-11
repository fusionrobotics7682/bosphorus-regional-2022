// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Teleop.binary.GetBallWithFeederCommand;
import frc.robot.commands.Teleop.binary.ShootWithFeederCommand;
import frc.robot.commands.Teleop.unit.Drive.TankDriveCommand;
import frc.robot.commands.Teleop.unit.Feeder.GetInFeederCommand;
import frc.robot.commands.Teleop.unit.Feeder.GetOutFeederCommand;
import frc.robot.commands.Teleop.unit.Intake.GetInTakeCommand;
import frc.robot.commands.Teleop.unit.Intake.GetOutTakeCommand;
import frc.robot.commands.Teleop.unit.Lift.LiftUpCommand;
import frc.robot.commands.Teleop.unit.Shooter.ShootInTarmacCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final FeederSubsystem feederSubsystem = new FeederSubsystem();
  private final LiftSubsystem liftSubsystem = new LiftSubsystem();

  // Controller
  Joystick joystick = new Joystick(0);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    driveSubsystem.setDefaultCommand(new TankDriveCommand(driveSubsystem));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Unit process
    new JoystickButton(joystick, 1).whileActiveContinuous(new GetInFeederCommand(feederSubsystem));
    new JoystickButton(joystick, 2).whileActiveContinuous(new GetOutFeederCommand(feederSubsystem));
    new JoystickButton(joystick, 3).whileActiveContinuous(new GetInTakeCommand(intakeSubsystem));
    new JoystickButton(joystick, 4).whileActiveContinuous(new GetOutTakeCommand(intakeSubsystem));
    new JoystickButton(joystick, 5).whileActiveContinuous(new ShootInTarmacCommand(shooterSubsystem));
    new JoystickButton(joystick, 6).whileActiveContinuous(new LiftUpCommand(liftSubsystem));

    // Parallel process
    new JoystickButton(joystick, 7).whileActiveContinuous(new GetBallWithFeederCommand(feederSubsystem, intakeSubsystem));
    new JoystickButton(joystick, 8).whileActiveContinuous(new ShootWithFeederCommand(feederSubsystem, shooterSubsystem));
    // for back
    // Use getOutFeederCommand and getOutTakeCommand
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}