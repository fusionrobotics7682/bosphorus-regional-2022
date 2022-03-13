// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Path;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class TwoBallPathCommand extends CommandBase {

  String trajectoryJSON = "PathWeaver/Paths/rapid_lower.json";
  Trajectory trajectory = new Trajectory();
  RamseteCommand ramseteCommand;

  DriveSubsystem driveSubsystem;

  /** Creates a new TwoBallPathCommand. */
  public TwoBallPathCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException e) {
      e.printStackTrace();
    }

    ramseteCommand =
    new RamseteCommand(
        trajectory,
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

        driveSubsystem.resetOdometry(trajectory.getInitialPose());

  }

  public Command getRamseteCommand(){
    return ramseteCommand;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {     
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
