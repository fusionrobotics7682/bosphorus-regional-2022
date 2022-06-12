// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Autonomous.polymorphic.ThreeBallScenarios.MidThreeBallCommand;
import frc.robot.commands.Autonomous.polymorphic.TwoBallScenarios.LowerTwoBallCommand;
import frc.robot.commands.Autonomous.polymorphic.TwoBallScenarios.MidTwoBallCommand;
import frc.robot.commands.Autonomous.polymorphic.TwoBallScenarios.UpperTwoBallCommand;
import frc.robot.commands.Path.TwoBallLowerPathCommand;
import frc.robot.commands.Teleop.Unit.Drive.*;
import frc.robot.commands.Teleop.Unit.Feeder.*;
import frc.robot.commands.Teleop.Unit.Intake.*;
import frc.robot.commands.Teleop.Unit.Lift.*;
import frc.robot.commands.Teleop.Unit.Shooter.*;
import frc.robot.commands.Teleop.Binary.GetBallWithFeederCommand;
import frc.robot.commands.Teleop.Binary.ShootWithFeederCommand;
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

  // Subsystems
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final FeederSubsystem feederSubsystem = new FeederSubsystem();
  private final LiftSubsystem liftSubsystem = new LiftSubsystem();

  // Commands

  // Path Commands

      // Two Ball Path Commands
      TwoBallLowerPathCommand twoBallPathCommand = new TwoBallLowerPathCommand(driveSubsystem);

  // Gyro|Encoder Commands

      // Two Ball Scenarios
      LowerTwoBallCommand lowerTwoBallCommand = new LowerTwoBallCommand(driveSubsystem, intakeSubsystem, feederSubsystem, shooterSubsystem);
      MidTwoBallCommand midTwoBallCommand = new MidTwoBallCommand(driveSubsystem, intakeSubsystem, feederSubsystem, shooterSubsystem);
      UpperTwoBallCommand upperTwoBallCommand = new UpperTwoBallCommand(driveSubsystem, intakeSubsystem, feederSubsystem, shooterSubsystem);

      // Three Ball Scenarios
      MidThreeBallCommand midThreeBallCommand = new MidThreeBallCommand(driveSubsystem, intakeSubsystem, feederSubsystem, shooterSubsystem);

  // Controller
  Joystick joystick = new Joystick(0);



  DifferentialDriveVoltageConstraint autoVoltageConstraint =
  new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
          Constants.DRIVE_CONSTANTS.KS_VOLTS,
          Constants.DRIVE_CONSTANTS.KV_VOLTS),
      driveSubsystem.getKinematics(),
      10);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    driveSubsystem.setDefaultCommand(new ArcadeDriveCommand(driveSubsystem));

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
    return midThreeBallCommand.getCommand();
     


    //    new DriveLeftXFeetEncoderCommand()
       //new DriveStraightEncoderCommand(driveSubsystem,) 
/*

 DifferentialDriveVoltageConstraint autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            Constants.DRIVE_CONSTANTS.KS_VOLTS,
            Constants.DRIVE_CONSTANTS.KV_VOLTS
            ),
        driveSubsystem.getKinematics(),
        10);
// Create config for trajectory
TrajectoryConfig config =
    new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

// An example trajectory to follow.  All units in meters.
Trajectory exampleTrajectory =
    TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config);

RamseteCommand ramseteCommand =
    new RamseteCommand(
        exampleTrajectory,
        m_robotDrive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(
            DriveConstants.ksVolts,
            DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        m_robotDrive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts,
        m_robotDrive);

// Reset odometry to the starting pose of the trajectory.
m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

// Run path following command, then stop at the end.
return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
}*/
      }


}