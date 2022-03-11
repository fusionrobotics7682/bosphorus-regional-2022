// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.Intake.TimerBased;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class GetInTakeXSecond extends CommandBase {

  IntakeSubsystem intakeSubsystem;
  Timer timer = new Timer();

  double seconds = 0;

  /** Creates a new GetInTakeXSecond. */
  public GetInTakeXSecond(IntakeSubsystem intakeSubsystem, double seconds) {
    this.intakeSubsystem = intakeSubsystem;
    this.seconds = seconds;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.getIn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.get() <= seconds){
      return false;
    }
      return true;
  }
}
