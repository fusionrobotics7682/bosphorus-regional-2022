// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.unit.Feeder.TimerBased;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;

public class GetFeederXSecond extends CommandBase {

  FeederSubsystem feederSubsystem;
  Timer timer = new Timer();

  double seconds = 0;
  
  /** Creates a new GetFeederXSecond. */
  public GetFeederXSecond(FeederSubsystem feederSubsystem, double seconds) {
    this.feederSubsystem = feederSubsystem;
    this.seconds = seconds;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feederSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feederSubsystem.getIn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feederSubsystem.stopMotor();
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
