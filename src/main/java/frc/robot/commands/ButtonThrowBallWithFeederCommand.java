// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ButtonThrowBallWithFeederCommand extends CommandBase {

  DigitalInput frontLimitSwitch;
  DigitalInput backLimitSwitch;

  private FeederSubsystem feederSubsystem;
  private ShooterSubsystem shooterSubsystem;

  /** Creates a new ThrowBallCommand. */
  public ButtonThrowBallWithFeederCommand() {
    addRequirements(feederSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("THROW BALL COMMAND INITIALIZED !!!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feederSubsystem.getIn();
    shooterSubsystem.shoot();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("THROW BALL COMMAND IS FINISHED WORKED !!!");
    if(frontLimitSwitch.get() == false && backLimitSwitch.get() == false){
      return true;
    }
    return false;
  }
}
