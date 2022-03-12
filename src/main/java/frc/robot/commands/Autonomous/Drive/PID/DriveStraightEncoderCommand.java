// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.Drive.PID;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveStraightEncoderCommand extends CommandBase {


  /** Creates a new DriveForward. */
  DriveSubsystem driveSubSystem;

  double feetSetpoint = 0;
  double outputSpeed = 0;
  boolean isBack = false;

  public DriveStraightEncoderCommand(DriveSubsystem driveSubSystem, boolean isBack, double feetSetpoint) {
      this.feetSetpoint = feetSetpoint;
      this.driveSubSystem = driveSubSystem;
      this.isBack = isBack;
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
    double lastRightError = driveSubSystem.getRightEncoderDistance() * Constants.DRIVE_CONSTANTS.K_DRIVE_TICK_2_FEET;
    double rightEncoder = driveSubSystem.getRightEncoderDistance();

    double sensorPosition = rightEncoder * Constants.DRIVE_CONSTANTS.K_DRIVE_TICK_2_FEET;

    double lastTimeStamp = 0;
    double dt = Timer.getFPGATimestamp() - lastTimeStamp;

    // calculations
    double error = feetSetpoint - sensorPosition;
    
    double errorRate = (error-lastRightError) / dt;
    
    outputSpeed = Constants.DRIVE_CONSTANTS.KP * error + Constants.DRIVE_CONSTANTS.KD * errorRate;

    if(isBack){
      outputSpeed*=-1;
    }

    driveSubSystem.setLeftMotorGroupSpeed(outputSpeed*0.25);
    driveSubSystem.setRightMotorGroupSpeed(outputSpeed*0.25);

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
