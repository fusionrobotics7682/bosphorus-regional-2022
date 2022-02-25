// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoGetBallCommand extends CommandBase {

  private IntakeSubsystem intakeSubsystem;
  private FeederSubsystem feederSubsystem;

  Timer timer = new Timer();

  //DigitalInput frontLimitSwitch;

  /** Creates a new ButtonGetBallCommand. */
  public AutoGetBallCommand(IntakeSubsystem intakeSubsystem, FeederSubsystem feederSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.feederSubsystem = feederSubsystem;
    addRequirements(intakeSubsystem);
    timer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("GET BALL COMMAND INITIALIZED !!!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get() <= 2){
      intakeSubsystem.getIn();
    }
    else if(timer.get() <= 2.5){
      intakeSubsystem.getIn();
      feederSubsystem.getIn();
    }
    else{
      intakeSubsystem.stopMotor();
      feederSubsystem.stopMotor();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*if(frontLimitSwitch.get() == true) {
      return true;
    }
    return false;
  }*/
  return false;
  }

}
