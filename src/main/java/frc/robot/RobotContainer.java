// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Autonomous.Drive.PID.DriveStraightEncoderCommand;
import frc.robot.commands.Autonomous.Drive.TimerBased.TurnLeftXsecond;
import frc.robot.commands.Autonomous.Drive.TimerBased.TurnRightXsecond;
import frc.robot.commands.Autonomous.Feeder.TimerBased.GetFeederXSecond;
import frc.robot.commands.Autonomous.Intake.TimerBased.GetInTakeXSecond;
import frc.robot.commands.Autonomous.Shooter.PID.ShootFromInTarmacCommand;
import frc.robot.commands.Autonomous.Shooter.PID.ShootFromOutTarmacCommand;
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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

    // 2 Ball Autonomous | For lower position
    /*return new SequentialCommandGroup(
    new ParallelCommandGroup(new DriveStraight(driveSubsystem, false, 3.5170603675), new GetInTakeXSecond(intakeSubsystem, 3)),
    new ParallelCommandGroup(new DriveStraight(driveSubsystem, true, 3.5170603675), new GetFeederXSecond(feederSubsystem, 1)), new TurnRightXsecond(driveSubsystem, 1.5),
    new ShootFromInTarmacCommand(shooterSubsystem, feederSubsystem));*/

    // 2 Ball Autonomous | For mid position
    /*return new SequentialCommandGroup(
    new ParallelCommandGroup(new DriveStraight(driveSubsystem, false, 3.5170603675), new GetInTakeXSecond(intakeSubsystem, 3)),
    new ParallelCommandGroup(new DriveStraight(driveSubsystem, true, 3.5170603675), new GetFeederXSecond(feederSubsystem, 1)), new TurnLeftXsecond(driveSubsystem, 1.5),
    new ShootFromInTarmacCommand(shooterSubsystem, feederSubsystem));*/

    // 2 Ball Autonomous | For higher position
    /*return new SequentialCommandGroup(
    new ParallelCommandGroup(new DriveStraight(driveSubsystem, false, 3.5170603675), new GetInTakeXSecond(intakeSubsystem, 3)),
    new ParallelCommandGroup(new DriveStraight(driveSubsystem, true, 3.5170603675), new GetFeederXSecond(feederSubsystem, 1)), new TurnRightXsecond(driveSubsystem, 1.5),
    new ShootFromInTarmacCommand(shooterSubsystem, feederSubsystem));*/

    // 3 Ball Autonomous | For mid position
    /*return new SequentialCommandGroup(
    new ParallelCommandGroup(new DriveStraight(driveSubsystem, false, 3.5170603675), new GetInTakeXSecond(intakeSubsystem, 3)),
    new ParallelCommandGroup(new DriveStraight(driveSubsystem, true, 4.5170603675), new GetFeederXSecond(feederSubsystem, 1)), 
    new TurnLeftXsecond(driveSubsystem, 1.5), new ShootFromInTarmacCommand(shooterSubsystem, feederSubsystem),
    new TurnRightXsecond(driveSubsystem, 1.5),
    new DriveStraight(driveSubsystem, false, 16.839), new GetInTakeXSecond(intakeSubsystem, 5), 
    new DriveStraight(driveSubsystem, true, 16.839), new TurnLeftXsecond(driveSubsystem, 1.5),
    new ShootFromOutTarmacCommand(shooterSubsystem, feederSubsystem));*/



    return null;
  }
}