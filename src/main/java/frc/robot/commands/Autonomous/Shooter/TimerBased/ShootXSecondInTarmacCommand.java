// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.Shooter.TimerBased;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootXSecondInTarmacCommand extends CommandBase {

  ShooterSubsystem shooterSubsystem;
  FeederSubsystem feederSubsystem;
  Timer timer = new Timer();

  /** Creates a new ShootToUpperHubCommand. */
  public ShootXSecondInTarmacCommand(ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem) {
    this.feederSubsystem = feederSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, feederSubsystem);
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
    shooterSubsystem.shoot(0.35, 0.70);
    if(timer.get() >= 1.5){
      feederSubsystem.getIn();
    }
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
