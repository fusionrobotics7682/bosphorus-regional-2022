// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.polymorphic.TwoBallScenarios;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autonomous.unit.Drive.PID.DriveStraightEncoderCommand;
import frc.robot.commands.Autonomous.unit.Drive.TimerBased.TurnRightXsecond;
import frc.robot.commands.Autonomous.unit.Feeder.TimerBased.GetFeederXSecond;
import frc.robot.commands.Autonomous.unit.Intake.TimerBased.GetInTakeXSecond;
import frc.robot.commands.Autonomous.unit.Shooter.PID.ShootFromInTarmacCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class LowerTwoBallCommand extends CommandBase {

  DriveSubsystem driveSubsystem;
  IntakeSubsystem intakeSubsystem;
  FeederSubsystem feederSubsystem;
  ShooterSubsystem shooterSubsystem;

  Command lowerTwoBallCommand;

  /** Creates a new LowerTwoBallCommand. */
  public LowerTwoBallCommand(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem,FeederSubsystem feederSubsystem, ShooterSubsystem shooterSubsystem){

    this.driveSubsystem = driveSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem, intakeSubsystem, feederSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lowerTwoBallCommand = new SequentialCommandGroup(
    new ParallelCommandGroup(new DriveStraightEncoderCommand(driveSubsystem, false, 3.5170603675), new GetInTakeXSecond(intakeSubsystem, 3)),
    new ParallelCommandGroup(new DriveStraightEncoderCommand(driveSubsystem, true, 3.5170603675), new GetFeederXSecond(feederSubsystem, 1)), new TurnRightXsecond(driveSubsystem, 1.5),
    new ShootFromInTarmacCommand(shooterSubsystem, feederSubsystem));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  public Command getCommand(){
    return lowerTwoBallCommand;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
