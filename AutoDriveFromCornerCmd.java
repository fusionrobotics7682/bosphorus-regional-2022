// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;

public class AutoDriveFromCornerCmd extends CommandBase {

  DriveSubsystem driveSubsystem;
  IntakeSubsystem intakeSubsystem;
  FeederSubsystem feederSubsystem;
  ShooterSubsystem shooterSubsystem;

  //public AHRS navx = new AHRS(Constants.DRIVE_CONSTANTS.NAVX_SPI_PORT);

  
  Timer timer = new Timer();

  /** Creates a new AutoDriveFromCornerCmd. */
  public AutoDriveFromCornerCmd(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem,
                                FeederSubsystem feederSubsystem, ShooterSubsystem shooterSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(driveSubsystem, intakeSubsystem, feederSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    navx.reset();
    navx.calibrate();
    navx.setAngleAdjustment(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Gyro:", navx.getAngle());
    intakeSubsystem.getOutChassis();
    
    if(timer.get() < 3){
      shooterSubsystem.shoot(-0.65, 0.40);
    }

    else if(timer.get() <= 6){
     feederSubsystem.getIn();
    }

    else if(timer.get() <= 9){
      intakeSubsystem.getIn();
      shooterSubsystem.shoot(-0.7, 0.45); // First argument: Rear, Second: Front
      driveSubsystem.autoDrive(0, 0.5); // Angle - Speed
    }

    else if(timer.get() <= 12){
      intakeSubsystem.getIn();
      feederSubsystem.getIn();
    }

  }

  // Called once the command ends or is interrupted.0
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
    intakeSubsystem.stopMotor();
    feederSubsystem.stopMotor();
    shooterSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.get() > 12){
      return true;
    }
    return false;
  }
}
