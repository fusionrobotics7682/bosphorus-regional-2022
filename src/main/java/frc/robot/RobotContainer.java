// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.Autonomous.polymorphic.ThreeBallScenarios.MidThreeBallCommand;
import frc.robot.commands.Autonomous.polymorphic.TwoBallScenarios.LowerTwoBallCommand;
import frc.robot.commands.Autonomous.polymorphic.TwoBallScenarios.MidTwoBallCommand;
import frc.robot.commands.Autonomous.polymorphic.TwoBallScenarios.UpperTwoBallCommand;
import frc.robot.commands.Path.TwoBallLowerPathCommand;
import frc.robot.commands.Teleop.binary.*;
import frc.robot.commands.Teleop.unit.Drive.ArcadeDriveCommand;
import frc.robot.commands.Teleop.unit.Feeder.GetInFeederCommand;
import frc.robot.commands.Teleop.unit.Feeder.GetOutFeederCommand;
import frc.robot.commands.Teleop.unit.Intake.GetInTakeCommand;
import frc.robot.commands.Teleop.unit.Intake.GetOutTakeCommand;
import frc.robot.commands.Teleop.unit.Lift.LiftUpCommand;
import frc.robot.commands.Teleop.unit.Shooter.ShootInTarmacCommand;
import frc.robot.commands.Teleop.unit.Drive.*;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
 
  FeederSubsystem feederSubsystem = new FeederSubsystem();
  DriveSubsystem driveSubsystem = new DriveSubsystem();
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  private RobotContainer robotContainer;

  RamseteCommand get1ballTrajectoryCommand;
  RamseteCommand getthirdballTrajectoryCommand;
  RamseteCommand throwfirstballsCommand;
  RamseteCommand throwsecondballsCommand;

  String get1ball = "path_outputs/output/get1Ball.wpilib.json";
  Trajectory get1ballTrajectory = new Trajectory();

  String getthirdball = "path_outputs/output/getthirdball.wpilib.json";
  Trajectory getthirdballTrajectory = new Trajectory();

  String throwfirstballs = "path_outputs/output/throwfirstballs.wpilib.json";
  Trajectory throwfirstballsTrajectory = new Trajectory();

  String throwsecondballs = "path_outputs/output/throwsecondballs.wpilib.json";
  Trajectory throwsecondballsTrajectory = new Trajectory();

  // Commands

  // Path Commands

      // Two Ball Path Commands
      TwoBallLowerPathCommand twoBallPathCommand = new TwoBallLowerPathCommand();

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

    driveSubsystem.setDefaultCommand(new TankDriveCommand(driveSubsystem));  
    
    try {
        Path get1ballpath = Filesystem.getDeployDirectory().toPath().resolve(get1ball);
        get1ballTrajectory = TrajectoryUtil.fromPathweaverJson(get1ballpath);
     } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + get1ball, ex.getStackTrace());
     }
  
     try {
      Path getthirdballpath = Filesystem.getDeployDirectory().toPath().resolve(getthirdball);
      get1ballTrajectory = TrajectoryUtil.fromPathweaverJson(getthirdballpath);
   } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + getthirdball, ex.getStackTrace());
   }
  
     try {
      Path throwfirstballsPath = Filesystem.getDeployDirectory().toPath().resolve(throwfirstballs);
      get1ballTrajectory = TrajectoryUtil.fromPathweaverJson(throwfirstballsPath);
   } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + throwfirstballs, ex.getStackTrace());
   }
  
     try {
      Path throwsecondballsPath = Filesystem.getDeployDirectory().toPath().resolve(throwsecondballs);
      throwsecondballsTrajectory = TrajectoryUtil.fromPathweaverJson(throwsecondballsPath);
   } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + throwsecondballs, ex.getStackTrace());
   }

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
   // return midThreeBallCommand.getCommand();
    //    new DriveLeftXFeetEncoderCommand()
       //new DriveStraightEncoderCommand(driveSubsystem,);
              // try this trajectoryJSON after exampleTrajectory
                      get1ballTrajectoryCommand =
                      new RamseteCommand(
                        get1ballTrajectory,
                      driveSubsystem::getPose,
                      new RamseteController(2.1, 0.8),
                      new SimpleMotorFeedforward(
                          Constants.DRIVE_CONSTANTS.KS_VOLTS,
                          Constants.DRIVE_CONSTANTS.KV_VOLTS
                          ),
                      driveSubsystem.getKinematics(),
                      driveSubsystem::getWheelSpeeds,
                      new PIDController(Constants.DRIVE_CONSTANTS.KP, Constants.DRIVE_CONSTANTS.KI, Constants.DRIVE_CONSTANTS.KD),
                      new PIDController(Constants.DRIVE_CONSTANTS.KP, Constants.DRIVE_CONSTANTS.KI, Constants.DRIVE_CONSTANTS.KD),
                      // RamseteCommand passes volts to the callback
                      driveSubsystem::tankDriveVolts,
                      driveSubsystem);
      
                      // try this trajectoryJSON after exampleTrajectory
                      getthirdballTrajectoryCommand =
                      new RamseteCommand(
                      getthirdballTrajectory,
                      driveSubsystem::getPose,
                      new RamseteController(2.1, 0.8),
                      new SimpleMotorFeedforward(
                          Constants.DRIVE_CONSTANTS.KS_VOLTS,
                          Constants.DRIVE_CONSTANTS.KV_VOLTS
                          ),
                      driveSubsystem.getKinematics(),
                      driveSubsystem::getWheelSpeeds,
                      new PIDController(Constants.DRIVE_CONSTANTS.KP, Constants.DRIVE_CONSTANTS.KI, Constants.DRIVE_CONSTANTS.KD),
                      new PIDController(Constants.DRIVE_CONSTANTS.KP, Constants.DRIVE_CONSTANTS.KI, Constants.DRIVE_CONSTANTS.KD),
                      // RamseteCommand passes volts to the callback
                      driveSubsystem::tankDriveVolts,
                      driveSubsystem);
      
                              // try this trajectoryJSON after exampleTrajectory
                      throwfirstballsCommand =
                      new RamseteCommand(
                        throwfirstballsTrajectory,
                      driveSubsystem::getPose,
                      new RamseteController(2.1, 0.8),
                      new SimpleMotorFeedforward(
                          Constants.DRIVE_CONSTANTS.KS_VOLTS,
                          Constants.DRIVE_CONSTANTS.KV_VOLTS
                          ),
                      driveSubsystem.getKinematics(),
                      driveSubsystem::getWheelSpeeds,
                      new PIDController(Constants.DRIVE_CONSTANTS.KP, Constants.DRIVE_CONSTANTS.KI, Constants.DRIVE_CONSTANTS.KD),
                      new PIDController(Constants.DRIVE_CONSTANTS.KP, Constants.DRIVE_CONSTANTS.KI, Constants.DRIVE_CONSTANTS.KD),
                      // RamseteCommand passes volts to the callback
                      driveSubsystem::tankDriveVolts,
                      driveSubsystem);
      
                      // try this trajectoryJSON after exampleTrajectory
                      throwsecondballsCommand =
                      new RamseteCommand(
                      throwsecondballsTrajectory,
                      driveSubsystem::getPose,
                      new RamseteController(2.1, 0.8),
                      new SimpleMotorFeedforward(
                          Constants.DRIVE_CONSTANTS.KS_VOLTS,
                          Constants.DRIVE_CONSTANTS.KV_VOLTS
                          ),
                      driveSubsystem.getKinematics(),
                      driveSubsystem::getWheelSpeeds,
                      new PIDController(Constants.DRIVE_CONSTANTS.KP, Constants.DRIVE_CONSTANTS.KI, Constants.DRIVE_CONSTANTS.KD),
                      new PIDController(Constants.DRIVE_CONSTANTS.KP, Constants.DRIVE_CONSTANTS.KI, Constants.DRIVE_CONSTANTS.KD),
                      // RamseteCommand passes volts to the callback
                      driveSubsystem::tankDriveVolts,
                      driveSubsystem);

                      return get1ballTrajectoryCommand.andThen(throwfirstballsCommand.andThen(getthirdballTrajectoryCommand.andThen(throwsecondballsCommand))) ;

  }
}
