// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.Drive.TimerBased;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnRightXsecond extends CommandBase {

  private DriveSubsystem driveSubSystem;
  private Timer timer = new Timer();

  private double seconds = 0;



  /** Creates a new TurnXsecond. */
  public TurnRightXsecond(DriveSubsystem driveSubsystem, double seconds) {
    this.driveSubSystem = driveSubSystem;
    this.seconds = seconds;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubSystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get() <= seconds){
    driveSubSystem.setLeftMotorGroupSpeed(0.5);
    driveSubSystem.setRightMotorGroupSpeed(-0.5);
    }
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
