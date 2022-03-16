// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Path.two_ball_path;

import edu.wpi.first.wpilibj2.command.CommandBase;
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
import frc.robot.Constants;
import frc.robot.commands.Autonomous.polymorphic.ThreeBallScenarios.MidThreeBallCommand;
import frc.robot.commands.Autonomous.polymorphic.TwoBallScenarios.LowerTwoBallCommand;
import frc.robot.commands.Autonomous.polymorphic.TwoBallScenarios.MidTwoBallCommand;
import frc.robot.commands.Autonomous.polymorphic.TwoBallScenarios.UpperTwoBallCommand;
import frc.robot.commands.Path.two_ball_path.TwoBallLowerPathCommand;
import frc.robot.commands.Teleop.binary.*;
import frc.robot.commands.Teleop.unit.Drive.ArcadeDriveCommand;
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
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class TwoBallMidPathCommand extends CommandBase {
  DriveSubsystem driveSubsystem;

  RamseteCommand get1ballCommand;
  RamseteCommand throwfirstballsCommand;
  RamseteCommand twoBallLowerPathCommand;

  String get1ball = "path_outputs/output/get1ball_mid_two.wpilib.json";
  Trajectory get1ballTrajectory = new Trajectory();

  String throwfirstballs = "path_outputs/output/getthrowball.wpilib.json";
  Trajectory throwfirstballsTrajectory = new Trajectory();

  DifferentialDriveVoltageConstraint autoVoltageConstraint =
  new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
          Constants.DRIVE_CONSTANTS.KS_VOLTS,
          Constants.DRIVE_CONSTANTS.KV_VOLTS),
      driveSubsystem.getKinematics(),
      10);

      

  /** Creates a new TwoBallLowerPathCommand. */
  public TwoBallMidPathCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
        
    try {
      Path  get1ballpath = Filesystem.getDeployDirectory().toPath().resolve(get1ball);
      get1ballTrajectory = TrajectoryUtil.fromPathweaverJson(get1ballpath);
   } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + get1ball, ex.getStackTrace());
   }

   try {
    Path throwfirstballsPath = Filesystem.getDeployDirectory().toPath().resolve(throwfirstballs);
    get1ballTrajectory = TrajectoryUtil.fromPathweaverJson(throwfirstballsPath);
 } catch (IOException ex) {
    DriverStation.reportError("Unable to open trajectory: " + throwfirstballs, ex.getStackTrace());
 }

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    get1ballCommand =          
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
      
  }

  public Command get1BallCommand() {
    return get1ballCommand;
  }

  public Command throwfirstballsCommand(){
    return throwfirstballsCommand;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
