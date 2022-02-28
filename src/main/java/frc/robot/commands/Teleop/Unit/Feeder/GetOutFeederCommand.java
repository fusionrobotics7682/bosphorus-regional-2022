// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop.Unit.Feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;

public class GetOutFeederCommand extends CommandBase {

  private FeederSubsystem feederSubsystem;

  /** Creates a new GetOutFeederCommand. */
  public GetOutFeederCommand(FeederSubsystem feederSubsystem) {
    this.feederSubsystem = feederSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feederSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feederSubsystem.getOut();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feederSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
