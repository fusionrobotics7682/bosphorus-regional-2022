// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnTo90Degrees extends CommandBase {

  DriveSubsystem driveSubsystem;
  AHRS gyro = new AHRS(SPI.Port.kMXP);

  /** Creates a new TurnTo360Degrees. */
  public TurnTo90Degrees() {
    addRequirements(driveSubsystem);
  }

  final double kP = 0.5;
  double setpoint = 90;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = setpoint - gyro.getAngle();
    double output = error * kP;
    driveSubsystem.arcadeDrive(0, output);
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
