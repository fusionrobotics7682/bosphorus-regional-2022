// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.unit.Shooter.Velocity;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// 3/4 of backShooter
// 4/4 of frontShooter

public class ShootXSpeedOutTarmacCommand extends CommandBase {

  private ShooterSubsystem shooterSubsystem;
  private double speedSetpoint;
  
  Timer timer = new Timer();

  /** Creates a new ShootXSpeedOutTarmacCommand. */
  public ShootXSpeedOutTarmacCommand(ShooterSubsystem shooterSubsystem, double speedSetpoint) {
    this.shooterSubsystem = shooterSubsystem;
    this.speedSetpoint = speedSetpoint;

    timer.reset();
    timer.start();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double frontFeet = shooterSubsystem.getFrontEncoder()*Constants.DRIVE_CONSTANTS.K_DRIVE_TICK_2_FEET;
    double rearFeet = shooterSubsystem.getRearEncoder()*Constants.DRIVE_CONSTANTS.K_DRIVE_TICK_2_FEET;

    double frontMeter = Units.feetToMeters(frontFeet);
    double rearMeter = Units.feetToMeters(rearFeet);

    double frontSpeed = frontMeter/timer.get();
    double rearSpeed = rearMeter/timer.get();

    double frontVelocityError = Constants.SHOOTER_CONSTANTS.FRONT_VELOCITY_SETPOINT-frontSpeed;
    double rearVelocityError = Constants.SHOOTER_CONSTANTS.REAR_VELOCITY_SETPOINT-rearSpeed;

    frontSpeed += frontVelocityError*Constants.SHOOTER_CONSTANTS.FRONT_VELOCITY_P;
    rearSpeed += rearVelocityError*Constants.SHOOTER_CONSTANTS.REAR_VELOCITY_P; 

    shooterSubsystem.setFrontSpeed(frontSpeed);
    shooterSubsystem.setRearSpeed(rearSpeed);
  
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
