// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {

  Victor feeder = new Victor(Constants.FEEDER_CONSTANTS.FEEDER_MOTOR_PIN);
  /** Creates a new FeederSubsystem. */
  public FeederSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Getting the ball
  
  public void getIn(){
    feeder.set(Constants.FEEDER_CONSTANTS.FEEDER_SPEED);
  }

  // Putting the ball
  public void getOut(){
    feeder.setInverted(true);
    feeder.set(Constants.FEEDER_CONSTANTS.FEEDER_SPEED);
  }

  public void stopMotor(){
    feeder.set(0);
  }

  public double getFeederSpeed(){
    return Constants.FEEDER_CONSTANTS.FEEDER_SPEED;
  }
  
}
