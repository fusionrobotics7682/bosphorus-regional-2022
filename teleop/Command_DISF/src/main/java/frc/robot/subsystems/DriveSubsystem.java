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

  Victor frontLeft = new Victor(Constants.DRIVE_CONSTANTS.FRONT_LEFT_MOTOR_PIN);
  Victor rearLeft = new Victor(Constants.DRIVE_CONSTANTS.REAR_LEFT_MOTOR_PIN);
  Victor frontRight = new Victor(Constants.DRIVE_CONSTANTS.FRONT_RIGHT_MOTOR_PIN);
  Victor rearRight = new Victor(Constants.DRIVE_CONSTANTS.REAR_RIGHT_MOTOR_PIN);

  AHRS NavX = new AHRS(Port.kMXP);

  public MotorControllerGroup leftMotorGroup = new MotorControllerGroup(rearLeft, frontLeft);
  public MotorControllerGroup rightMotorGroup = new MotorControllerGroup(rearRight, frontRight);

  DifferentialDrive drive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
  double error;
  double setpoint = 10;

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

  public void pidForward(){
    double error = 0;
    error = setpoint - NavX.getAngle();
    double output = error * Constants.DRIVE_CONSTANTS.KP;
    drive.arcadeDrive(output, 0);
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
