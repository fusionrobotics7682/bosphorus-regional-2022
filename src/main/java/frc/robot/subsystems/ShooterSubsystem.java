// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  Timer timer = new Timer();
  
  Victor frontShooter = new Victor(Constants.SHOOTER_CONSTANTS.FRONT_SHOOTER_MOTOR_PIN);
  Victor rearShooter = new Victor(Constants.SHOOTER_CONSTANTS.REAR_SHOOTER_MOTOR_PIN);

  Encoder frontEncoder = new Encoder(Constants.SHOOTER_CONSTANTS.FRONT_ENCODER_CHANNEL_A, Constants.SHOOTER_CONSTANTS.FRONT_ENCODER_CHANNEL_B);
  Encoder rearEncoder = new Encoder(Constants.SHOOTER_CONSTANTS.REAR_ENCODER_CHANNEL_A, Constants.SHOOTER_CONSTANTS.REAR_ENCODER_CHANNEL_B);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    rearShooter.setInverted(true);
    frontShooter.setInverted(true);
  }

  @Override
  public void periodic() {
   
  }

  public void shoot(double rearSpeed, double frontSpeed) {
    rearShooter.set(rearSpeed);
    frontShooter.set(frontSpeed);
  }

  public void getInSlower(){
    rearShooter.setInverted(true);
    frontShooter.setInverted(true);
    rearShooter.set(0.75);
    frontShooter.set(0.75);
  }

  public void stopMotor() {
    rearShooter.set(0);
    frontShooter.set(0);
  }

  public double getFrontEncoder() {
    return frontEncoder.get();
  }
  
  public double getRearEncoder() {
    return rearEncoder.get();
  }

  public double getShooterSpeed(){
    return Constants.SHOOTER_CONSTANTS.SHOOTER_SPEED;
  }

  public void resetEncoders() {
    frontEncoder.reset();
    rearEncoder.reset();
  }

  
}
