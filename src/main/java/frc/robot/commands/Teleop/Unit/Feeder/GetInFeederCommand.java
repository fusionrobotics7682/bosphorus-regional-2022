// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop.Unit.Feeder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;

public class GetInFeederCommand extends CommandBase {

  private FeederSubsystem feederSubsystem;
  private DigitalInput frontLimitSwitch;

  /** Creates a new GetInFeederCommand. */
  public GetInFeederCommand(FeederSubsystem feederSubsystem) {
    this.feederSubsystem = feederSubsystem;
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
    /*if(frontLimitSwitch.get() == false){
    return false;
    }else{
      return true;
    }*/
    return false;
  }
}
