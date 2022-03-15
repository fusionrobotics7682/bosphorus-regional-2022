// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.unit.Drive.PID;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveForwardXFeetCommand extends CommandBase {

  DriveSubsystem driveSubSystem;

  double feetSetpoint = 0;
  double outputSpeed = 0;
  double lastRightError = 0;
 
  public DriveForwardXFeetCommand(DriveSubsystem driveSubSystem, double feetSetpoint) {
      this.feetSetpoint = feetSetpoint;
      this.driveSubSystem = driveSubSystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubSystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.driveSubSystem.resetEncoders();
    this.driveSubSystem.resetGyro();
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double rightEncoderPosition = driveSubSystem.getRightEncoderDistance() * Constants.DRIVE_CONSTANTS.K_DRIVE_TICK_2_FEET;
 
    double lastTimeStamp = 0;
    double dt = Timer.getFPGATimestamp() - lastTimeStamp;
 
    // calculations
    double error = feetSetpoint - rightEncoderPosition;
    double errorRate = (error-lastRightError) / dt;
    
    outputSpeed = Constants.DRIVE_CONSTANTS.KP * error + Constants.DRIVE_CONSTANTS.KD * errorRate;
 
 
    // set motors
    driveSubSystem.setLeftMotorGroupSpeed(outputSpeed*0.25);
    driveSubSystem.setRightMotorGroupSpeed(outputSpeed*0.25);
 
    // Pass last values
    lastRightError = error;
    lastTimeStamp = Timer.getFPGATimestamp();
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
