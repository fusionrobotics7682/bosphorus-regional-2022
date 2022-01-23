// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Victor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  Victor frontLeft = new Victor(0);
  Victor rearLeft = new Victor(0);
  Victor frontRight = new Victor(0);
  Victor rearRight = new Victor(0);

  MotorControllerGroup leftMotorGroup = new MotorControllerGroup(rearLeft, frontLeft);
  MotorControllerGroup rightMotorGroup = new MotorControllerGroup(rearRight, frontRight);

  DifferentialDrive drive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

  /** Creates a new DriveCommand. */
  public DriveSubsystem() {
  }

  @Override
  public void periodic() {
  }

  public void arcadeDrive(){
    drive.arcadeDrive(0.5, 0.5);
  }

  public void forwardDrive(){
    leftMotorGroup.set(0.5);
    rightMotorGroup.set(0.5);
  }

  public void backwardDrive(){
    leftMotorGroup.set(-0.5);
    rightMotorGroup.set(-0.5);
  }
}
