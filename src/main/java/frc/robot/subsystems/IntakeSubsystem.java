// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  DoubleSolenoid doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);

  Victor intake = new Victor(Constants.INTAKE_CONSTANTS.INTAKE_MOTOR_PIN);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void getIn(){
    intake.set(1);
    doubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void getInTake(){
    doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void getOut(){
    intake.setInverted(true);
    intake.set(1);
    //doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void stopMotor(){
    intake.set(0);
  }

  public double getIntakeSpeed(){
    return Constants.INTAKE_CONSTANTS.INTAKE_SPEED;
  }

}
