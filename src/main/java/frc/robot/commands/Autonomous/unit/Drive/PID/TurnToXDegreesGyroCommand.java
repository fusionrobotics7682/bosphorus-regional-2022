// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.unit.Drive.PID;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

// Giving | exit due to code | error
public class TurnToXDegreesGyroCommand extends CommandBase {

  private DriveSubsystem driveSubSystem;

  double gyroError = 0;
  double gyroSetpoint = 0;
  double outputSpeed = 0;

  /** Creates a new TurnToXDegrees. */
  public TurnToXDegreesGyroCommand(DriveSubsystem driveSubSystem, double gyroSetpoint) {
    this.driveSubSystem = driveSubSystem;
    this.gyroSetpoint = gyroSetpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.driveSubSystem.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gyroError = gyroSetpoint - driveSubSystem.getYaw();
    
    outputSpeed = gyroError * Constants.DRIVE_CONSTANTS.GYRO_KP;
  
    driveSubSystem.setLeftMotorGroupSpeed(outputSpeed*0.5);
    driveSubSystem.setRightMotorGroupSpeed(outputSpeed*0.43);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubSystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
