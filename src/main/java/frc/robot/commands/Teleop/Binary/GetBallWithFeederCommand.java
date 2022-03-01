// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop.Binary;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class GetBallWithFeederCommand extends CommandBase {

  private FeederSubsystem feederSubsystem;
  private IntakeSubsystem intakeSubsystem;

  DigitalInput frontLimitSwitch;
  DigitalInput backLimitSwitch;
  

  /** Creates a new GetBallCommand. */
  public GetBallWithFeederCommand(FeederSubsystem feederSubsystem, IntakeSubsystem intakeSubsystem) {
    addRequirements(feederSubsystem, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("GET BALL COMMAND INITIALIZED !!!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("GET BALL COMMAND EXECUTED !!!");
    intakeSubsystem.getIn();
    feederSubsystem.getIn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feederSubsystem.stopMotor();
    intakeSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(frontLimitSwitch.get() == true && backLimitSwitch.get() == true || backLimitSwitch.get() == true){
      return true;
    }
    return false;
  }
}
