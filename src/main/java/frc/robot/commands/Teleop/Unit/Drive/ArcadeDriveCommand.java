// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop.Unit.Drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDriveCommand extends CommandBase {
  
  private DriveSubsystem driveSubsystem;
  Timer timer = new Timer();
  double xSpeed;
  double zSpeed;
  double xAxis;
  double zAxis;

  public ArcadeDriveCommand(DriveSubsystem driveSubsystem,double xSpeed, double zSpeed, double xAxis, double zAxis) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }
    

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("DriveCommand initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(xSpeed < 0.8 && zSpeed < 0.8){
    driveSubsystem.arcadeDrive(xSpeed*xAxis, zSpeed*zAxis);
    }
    else{
      driveSubsystem.arcadeDrive(0,0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
