// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  //Basic Victors
  Victor frontLeft = new Victor(Constants.DRIVE_CONSTANTS.FRONT_LEFT_MOTOR_PIN);
  Victor frontRight = new Victor(Constants.DRIVE_CONSTANTS.FRONT_RIGHT_MOTOR_PIN);
  Victor rearLeft = new Victor(Constants.DRIVE_CONSTANTS.REAR_LEFT_MOTOR_PIN);
  Victor rearRight = new Victor(Constants.DRIVE_CONSTANTS.REAR_RIGHT_MOTOR_PIN);

  // Motor Groups
  MotorControllerGroup leftMotorGroup = new MotorControllerGroup(frontLeft, rearLeft);
  MotorControllerGroup rightMotorGroup = new MotorControllerGroup(frontRight, rearRight);

  // Sensors
  Encoder leftEncoder = new Encoder(Constants.DRIVE_CONSTANTS.LEFT_DRIVE_ENCODER_A_CHANNEL, Constants.DRIVE_CONSTANTS.LEFT_DRIVE_ENCODER_B_CHANNEL);
  Encoder rightEncoder = new Encoder(Constants.DRIVE_CONSTANTS.RIGHT_DRIVE_ENCODER_A_CHANNEL, Constants.DRIVE_CONSTANTS.RIGHT_DRIVE_ENCODER_B_CHANNEL);
  AHRS navx = new AHRS(Constants.DRIVE_CONSTANTS.NAVX_SPI_PORT);

  // Drive
  public DifferentialDrive differentialDrive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

  // Controller
  Joystick joystick = new Joystick(0);

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void arcadeDrive() {
    differentialDrive.arcadeDrive(joystick.getY()*Constants.DRIVE_CONSTANTS.ARCADE_DRIVE_Y_SPEED, joystick.getX()*Constants.DRIVE_CONSTANTS.ARCADE_DRIVE_X_SPEED);
  }

  public void tankDrive(){
    differentialDrive.tankDrive(joystick.getRawAxis(5)*Constants.DRIVE_CONSTANTS.TANK_DRIVE_LEFT_SPEED, joystick.getRawAxis(1)*Constants.DRIVE_CONSTANTS.TANK_DRIVE_RIGHT_SPEED);
  }

  public void setLeftMotorGroupSpeed(double speed) {
    leftMotorGroup.set(speed);
  }

  public void setRightMotorGroupSpeed(double speed) {
    rightMotorGroup.set(speed);
  }

  public double getLeftEncoderDistance() {
   return leftEncoder.getDistance();
  }

  public double getRightEncoderDistance() {
    return rightEncoder.getDistance();
  }

  public void resetEncoders(){
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public void resetGyro(){
    navx.reset();
  }

  public double getYaw(){
    return navx.getYaw();
  }

  public void stop(){
    differentialDrive.stopMotor();
  }

}
