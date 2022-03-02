// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.Autonomous.PID.StraightDriveGyroCommand;
import frc.robot.commands.Autonomous.PID.TurnTo360DegreesCommand;
import frc.robot.commands.Teleop.ArcadeDriveCommand;
import frc.robot.commands.Teleop.Unit.GetBallIntakeCommand;
import frc.robot.commands.Teleop.Unit.GetInFeederCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
  private final FeederSubsystem feederSubsystem = new FeederSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  Joystick joystick = new Joystick(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
    driveSubsystem.setDefaultCommand(new ArcadeDriveCommand(driveSubsystem, 0.5, joystick.getX(), 0.5, joystick.getZ()));

  }


  private void configureButtonBindings() {

    // Running sequential
    /*
    new JoystickButton(joystick, 5).whenActive(new ButtonGetBallWithFeederCommand());
    new JoystickButton(joystick, 6).whenActive(new ButtonThrowBallCommand(feederSubsystem, shooterSubsystem));
    */
    // Running Parallel
    new JoystickButton(joystick, 2).whileActiveContinuous(new ParallelCommandGroup(new GetBallIntakeCommand(intakeSubsystem),new GetInFeederCommand(feederSubsystem)));
  }



  public Command getAutonomousCommand() {
    
    // 1 Ball point and returning terminal
    // return new SequentialCommandGroup(new ButtonGoUpperHubPosition(), new ButtonThrowBallToUpperHub(), new ButtonGoTerminalCommand());

    // 2 Ball point  and returning terminal
    //return new SequentialCommandGroup(new ButtonGetTheReadyBall(), new ButtonGoUpperHubPosition(), new ButtonThrowBallToUpperHub(), new ButtonGoTerminalCommand());

    // 1 Ball point
    //return new SequentialCommandGroup(new ButtonThrowBallCommand());
    // return new StraightDriveGyroCommand(driveSubsystem);
    // return new StraightDriveGyroCommand(driveSubsystem);
    return new TurnTo360DegreesCommand(driveSubsystem);

  }
}
