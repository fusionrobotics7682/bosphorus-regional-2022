// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  Encoder leftEncoder = new Encoder(3, 1, true,  EncodingType.k4X);
  Encoder rightEncoder = new Encoder(4, 5, true, EncodingType.k4X);

  DifferentialDrive drive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
  double gyroError = 0;
  double encoderError = 0;
  double encoderSetpoint = 10;
  double gyroSetpoint = 90;
  final double K_DRIVE_TICK_2_FEET = 1.0 / 128 * 6 * Math.PI / 12;

  // All methods will in a loop
  /** Creates a new DriveCommand. */
  public DriveSubsystem() {
    NavX.reset();
    rightMotorGroup.setInverted(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Encoder Setpoint :", encoderSetpoint);
    SmartDashboard.putNumber("Get Current Encoder position :", leftEncoder.get()*K_DRIVE_TICK_2_FEET);
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
    gyroSetpoint = 360;
    gyroError = 0;
    gyroError = encoderSetpoint - NavX.getAngle();
    drive.arcadeDrive(0, gyroError*Constants.DRIVE_CONSTANTS.KP);
  }

  public void turnTo90Degrees(){
    encoderError = gyroSetpoint - NavX.getAngle();
    double outputSpeed = encoderError * Constants.DRIVE_CONSTANTS.KP;
    leftMotorGroup.set(outputSpeed);
    rightMotorGroup.set(outputSpeed);
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

  public void straightForwardPID(){

    double sensorPosition = rightEncoder.get() * Constants.DRIVE_CONSTANTS.K_DRIVE_TICK_2_FEET;

    // calculations
    double error = encoderSetpoint - sensorPosition;
    double outputSpeed = Constants.DRIVE_CONSTANTS.KP * error;

    drive.tankDrive(outputSpeed, outputSpeed);
  }

  public void onlyStraightForward(){
    double error = leftEncoder.get() - rightEncoder.get();

    double outputSpeed = Constants.DRIVE_CONSTANTS.KP * error;

    leftMotorGroup.set(-outputSpeed);
    rightMotorGroup.set(outputSpeed);
  }

  public double getTankDriveLeftMotorsSpeed(){
      return leftMotorGroup.get();
  }

  public double getTankDriveRightMotorsSpeed(){
      return rightMotorGroup.get();
  }

  public double getArcadeDriveXSpeed(){
      return Constants.DRIVE_CONSTANTS.ARCADE_DRIVE_Z_SPEED;
  }

  public double getArcadeDriveZSpeed(){
      return Constants.DRIVE_CONSTANTS.ARCADE_DRIVE_Z_SPEED;
  }

}
