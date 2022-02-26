// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AxisArcadeDriveCommand;
import frc.robot.commands.ButtonForwardDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
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
  Joystick joystick = new Joystick(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
    driveSubsystem.setDefaultCommand(new AxisArcadeDriveCommand(driveSubsystem, 0.5, joystick.getX(), 0.5, joystick.getZ()));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a 
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Running sequential
    /*new JoystickButton(joystick, 5).whenActive(new ButtonGetBallWithFeederCommand());
    new JoystickButton(joystick, 6).whenActive(new ThrowBallWithFeederCommand());

    // Running Parallel
    new JoystickButton(joystick, 1).whenActive(new ParallelCommandGroup(new ButtonGetBallCommand(), new ButtonThrowBallCommand()));
    new JoystickButton(joystick, 2).whenActive(new ParallelCommandGroup(new ButtonThrowBallCommand(), new GetInFeederCommand()));

    // Go a placement with path planning

    // Target : Hangar
    new JoystickButton(joystick, 3).whenActive(new ButtonGoHangar());

    // Target : Upper Hub Position
    new JoystickButton(joystick, 4).whenActive(new ButtonGoUpperHubPosition());

    // Target : Lower Hub Position
    new JoystickButton(joystick, 7).whenActive(new ButtonGoLowerHubPosition());
*/
    // Unit Process

   /* // Shooter Mechanism
    new JoystickButton(joystick, 8).whenActive(new ButtonThrowBallCommand());

    // Intake Mechanism
    new JoystickButton(joystick, 9).whenActive(new ButtonGetBallCommand());

    // Feeder Mechansim
    new JoystickButton(joystick, 10).whenActive(new GetInFeederCommand());
    */

    new JoystickButton(joystick, 4).whileActiveContinous(new ButtonForwardDriveCommand());
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    // 1 Ball point and returning terminal
    // return new SequentialCommandGroup(new ButtonGoUpperHubPosition(), new ButtonThrowBallToUpperHub(), new ButtonGoTerminalCommand());

    // 2 Ball point  and returning terminal
    //return new SequentialCommandGroup(new ButtonGetTheReadyBall(), new ButtonGoUpperHubPosition(), new ButtonThrowBallToUpperHub(), new ButtonGoTerminalCommand());

    // 1 Ball point
    //return new SequentialCommandGroup(new ButtonThrowBallCommand());
    return null;
  }
}
