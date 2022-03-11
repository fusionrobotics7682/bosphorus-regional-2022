// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* Knowlodge of Compressor and DoubleSolenoid
 * How Lift works?
*/
public class LiftSubsystem extends SubsystemBase {

  public Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  public DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);

  /** Creates a new LiftSubsystem. */
  public LiftSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void enableCompressor() {
    compressor.enableDigital();
  }

  public void disableCompressor() {
    compressor.disable();
  }

  public void liftUp() {
    solenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void liftDown() {
    solenoid.set(DoubleSolenoid.Value.kReverse);
  }
}
