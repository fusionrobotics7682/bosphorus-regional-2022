// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Path;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class SPathCommand extends CommandBase {
  // THIS CLASS CONTAINS S PATH
  String trajectoryJSON = "path_outputs/lower/twoballLowerGoBall1.path";
  Trajectory trajectory = new Trajectory();

  RamseteCommand ramseteCommand;
  DriveSubsystem driveSubsystem;

  /** Creates a new TwoBallPathCommand. */
  public SPathCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DifferentialDriveVoltageConstraint autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            Constants.DRIVE_CONSTANTS.KS_VOLTS,
            Constants.DRIVE_CONSTANTS.KV_VOLTS),
            driveSubsystem.getKinematics(),
            10.5);

// Create config for trajectory
TrajectoryConfig config =
    new TrajectoryConfig(
            10,
            12)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(driveSubsystem.getKinematics())
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

        // try this trajectoryJSON after exampleTrajectory
        ramseteCommand =
        new RamseteCommand(
        exampleTrajectory,
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

        // Reset odometry to the starting pose of the trajectory.
        driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        
}

  public Command getCommand(){
    return ramseteCommand;
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
