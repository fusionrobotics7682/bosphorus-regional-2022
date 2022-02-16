// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Victor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  Victor frontLeft = new Victor(1);
  Victor rearLeft = new Victor(2);
  Victor frontRight = new Victor(3);
  Victor rearRight = new Victor(4);

  AHRS NavX = new AHRS(Port.kMXP);

  public MotorControllerGroup leftMotorGroup = new MotorControllerGroup(rearLeft, frontLeft);
  public MotorControllerGroup rightMotorGroup = new MotorControllerGroup(rearRight, frontRight);

  DifferentialDrive drive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
  double error;
  double setpoint;

  // All methods will in a loop
  /** Creates a new DriveCommand. */
  public DriveSubsystem() {
    NavX.reset();
    rightMotorGroup.setInverted(true);
  }

  @Override
  public void periodic() {
  }

  public void arcadeDrive(double x, double z) {
    drive.arcadeDrive(x, z);
  }

  public void forwardDrive(){
    leftMotorGroup.set(0.5);
    rightMotorGroup.set(0.5);
  }

  public void backwardDrive(){
    leftMotorGroup.set(-0.5);
    rightMotorGroup.set(-0.5);
  }

  public void rightDrive(){
    leftMotorGroup.set(-0.5);
    rightMotorGroup.set(0.5);
  }

  public void leftDrive(){
    leftMotorGroup.set(0.5);
    rightMotorGroup.set(-0.5);
  }

  public void stopDrive(){
    leftMotorGroup.set(0);
    rightMotorGroup.set(0);
  }

  public void turnTo360degree(){
    setpoint = 360;
    error = 0;
    error = setpoint - NavX.getAngle();
    drive.arcadeDrive(0, error*Constants.DRIVE_CONSTANTS.KP);
  }

  public void goUpperHub (){
    // Path Planning code
  }

  public void goLowerHub(){
    // Path Planning code
  }

  public void goHangar(){
    // Path Planning code
  }
}
