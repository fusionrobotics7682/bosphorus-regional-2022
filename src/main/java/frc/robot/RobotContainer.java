// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.Teleop.Binary.GetBallWithFeederCommand;
import frc.robot.commands.Teleop.Binary.ThrowBallWithFeederCommand;
import frc.robot.commands.Teleop.Unit.Drive.ArcadeDriveCommand;
import frc.robot.commands.Teleop.Unit.Feeder.GetInFeederCommand;
import frc.robot.commands.Teleop.Unit.Feeder.GetOutFeederCommand;
import frc.robot.commands.Teleop.Unit.Intake.GetBallIntakeCommand;
import frc.robot.commands.Teleop.Unit.Intake.GetOutIntakeCommand;
import frc.robot.commands.Teleop.Unit.Shooter.GetInBallShooterCommand;
import frc.robot.commands.Teleop.Unit.Shooter.ThrowBallShooterCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
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
  private final FeederSubsystem feederSubsystem = new FeederSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  Joystick joystick = new Joystick(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
    driveSubsystem.setDefaultCommand(new ArcadeDriveCommand(driveSubsystem));

  }


  private void configureButtonBindings() {

    // Running Binary Commands

    /*
    new JoystickButton(joystick, 2).whileActiveContinuous(new GetBallWithFeederCommand(feederSubsystem, intakeSubsystem));
    new JoystickButton(joystick, 3).whileActiveContinuous(new ThrowBallWithFeederCommand(feederSubsystem, shooterSubsystem));
    */

    // Running Unit Commands
    // Intake
    new JoystickButton(joystick, 1).whileActiveContinuous(new GetBallIntakeCommand(intakeSubsystem));
    new JoystickButton(joystick, 4).whileActiveContinuous(new GetOutIntakeCommand(intakeSubsystem));
    // Feeder
    new JoystickButton(joystick, 2).whileActiveContinuous(new GetInFeederCommand(feederSubsystem));
    new JoystickButton(joystick, 3).whileActiveContinuous(new GetOutFeederCommand(feederSubsystem));
    /*
    // Shooter
    new JoystickButton(joystick, 5).whileActiveContinuous(new ThrowBallShooterCommand(shooterSubsystem));
    new JoystickButton(joystick, 6).whileActiveContinuous(new GetInBallShooterCommand(shooterSubsystem));
    */
    // Drive
    /* I did not write this because it is unneccessary but I left it here for everything */
    
  }



  public Command getAutonomousCommand() {

    return null;
  }
}
