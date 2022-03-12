// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.Drive.PID;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveRightXFeetEncoderCommand extends CommandBase {

  /** Creates a new DriveRightXFeetEncoderCommand. */
  DriveSubsystem driveSubsystem;
  double feetSetpoint = 0.375;

  /** Creates a new DriveLeftXFeetEncoderCommand. */
  public DriveRightXFeetEncoderCommand(DriveSubsystem driveSubsystem, double feetSetpoint) {
    this.driveSubsystem = driveSubsystem;
    this.feetSetpoint = feetSetpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.driveSubsystem.resetEncoders();
    this.driveSubsystem.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double leftEncoder = this.driveSubsystem.getLeftEncoderDistance();
    double rightEncoder = this.driveSubsystem.getRightEncoderDistance();

    double leftEncoderPosition = leftEncoder * Constants.DRIVE_CONSTANTS.K_DRIVE_TICK_2_FEET;
    double rightEncoderPosition = rightEncoder * Constants.DRIVE_CONSTANTS.K_DRIVE_TICK_2_FEET;

    double leftEncoderError = feetSetpoint - leftEncoderPosition;
    double rightEncoderError = feetSetpoint - rightEncoderPosition;

    double leftMotorOutput = leftEncoderError * Constants.DRIVE_CONSTANTS.KP;
    double rightMotorOutput = rightEncoderError * Constants.DRIVE_CONSTANTS.KP;

    this.driveSubsystem.setLeftMotorGroupSpeed(leftMotorOutput);
    this.driveSubsystem.setRightMotorGroupSpeed(-rightMotorOutput);

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
