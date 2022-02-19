// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  Victor shooter = new Victor(7);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {}

  @Override
  public void periodic() {
   
  }

  public void shoot() {
    shooter.set(1);
  }

  public void getInSlower(){
    shooter.setInverted(true);
    shooter.set(0.5);
  }

  public void stopMotor() {
    shooter.set(0);
  }
}
