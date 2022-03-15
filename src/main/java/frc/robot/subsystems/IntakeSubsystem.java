
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  Victor intake = new Victor(Constants.INTAKE_CONSTANTS.INTAKE_MOTOR_PIN);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void getIn(){
    intake.set(1);
  }

  public void getOut(){
    intake.setInverted(true);
    intake.set(Constants.FEEDER_CONSTANTS.FEEDER_SPEED);
  }

  public void stopMotor(){
    intake.set(0);
  }

  public double getIntakeSpeed(){
    return Constants.INTAKE_CONSTANTS.INTAKE_SPEED;
  }

}
