// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  //Basic Victors
  Victor frontLeft = new Victor(Constants.DRIVE_CONSTANTS.FRONT_LEFT_MOTOR_PIN);
  Victor frontRight = new Victor(Constants.DRIVE_CONSTANTS.FRONT_RIGHT_MOTOR_PIN);
  Victor rearLeft = new Victor(Constants.DRIVE_CONSTANTS.REAR_LEFT_MOTOR_PIN);
  Victor rearRight = new Victor(Constants.DRIVE_CONSTANTS.REAR_RIGHT_MOTOR_PIN);

  // Motor Groups
  public MotorControllerGroup leftMotorGroup = new MotorControllerGroup(rearLeft);
  public MotorControllerGroup rightMotorGroup = new MotorControllerGroup(rearLeft);

  // Sensors 10-20
  Encoder leftEncoder = new Encoder(Constants.DRIVE_CONSTANTS.LEFT_DRIVE_ENCODER_A_CHANNEL, Constants.DRIVE_CONSTANTS.LEFT_DRIVE_ENCODER_B_CHANNEL);
  Encoder rightEncoder = new Encoder(Constants.DRIVE_CONSTANTS.RIGHT_DRIVE_ENCODER_A_CHANNEL, Constants.DRIVE_CONSTANTS.RIGHT_DRIVE_ENCODER_B_CHANNEL);
  AHRS navx = new AHRS(Constants.DRIVE_CONSTANTS.NAVX_SPI_PORT);
  AnalogGyro gyro = new AnalogGyro(0);
  
  // SIM sensors
  EncoderSim leftEncoderSim= new EncoderSim(leftEncoder);
  EncoderSim rightEncoderSim= new EncoderSim(rightEncoder);

  // PID Controllers
  PIDController leftPIDController = new PIDController(Constants.DRIVE_CONSTANTS.KP, Constants.DRIVE_CONSTANTS.KI, Constants.DRIVE_CONSTANTS.KD);
  PIDController rightPIDController = new PIDController(Constants.DRIVE_CONSTANTS.KP, Constants.DRIVE_CONSTANTS.KI, Constants.DRIVE_CONSTANTS.KD);

  // Simple Motor Feedforward
  SimpleMotorFeedforward simpleMotorFeedforward = new SimpleMotorFeedforward(Constants.DRIVE_CONSTANTS.KS_VOLTS, Constants.DRIVE_CONSTANTS.KV_VOLTS);

  // Drive
  public DifferentialDrive differentialDrive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

  // Field Image
  Field2d field = new Field2d();

  // Controller
  Joystick joystick = new Joystick(0);

  //Odometry, kinematics and Pose Classes for trajectory
  Pose2d pose = new Pose2d();
  Rotation2d rotation2d = new Rotation2d();
  private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(rotation2d, new Pose2d(new Translation2d(8,8), new Rotation2d(5,-3.5)));
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.DRIVE_CONSTANTS.kTRACK_WIDTH);

  final double KvLinear = 1.98;
  final double KaLinear = 0.2;
  final double KvAngular = 1.5;
  final double KaAngular = 0.3;
    
  
// Create the simulation model of our drivetrain.
private DifferentialDrivetrainSim driveSim = DifferentialDrivetrainSim.createKitbotSim(
  KitbotMotor.kDualCIMPerSide, // 2 CIMs per side.
  KitbotGearing.k10p71,        // 10.71:1
  KitbotWheelSize.SixInch,     // 6" diameter wheels.
  null                         // No measurement noise.
);

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    // Invert the right motor group so that positive values on the joystick
    rightMotorGroup.setInverted(true);

    // Set the encoder distance per pulse
    leftEncoder.setDistancePerPulse(2 * Math.PI * Constants.DRIVE_CONSTANTS.kWHEEL_RADIUS / Constants.DRIVE_CONSTANTS.kENCODER_RESOLUTION);
    rightEncoder.setDistancePerPulse(2 * Math.PI * Constants.DRIVE_CONSTANTS.kWHEEL_RADIUS / Constants.DRIVE_CONSTANTS.kENCODER_RESOLUTION);

    // Reset the sensors
    resetEncoders();
    resetGyro();

    SmartDashboard.putData("Field", field);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(navx.getRotation2d() , leftEncoderSim.getDistance(), rightEncoderSim.getDistance());
    field.setRobotPose(odometry.getPoseMeters());
    SmartDashboard.putData("Field", field);
  }

  @Override
  public void simulationPeriodic() {
    driveSim.setInputs(leftMotorGroup.get() * RobotController.getInputVoltage(),
    rightMotorGroup.get() * RobotController.getInputVoltage());

    driveSim.update(0.02);
    odometry.update(navx.getRotation2d() , leftEncoderSim.getDistance(), rightEncoderSim.getDistance());
    field.setRobotPose(driveSim.getPose());
    SmartDashboard.putData("Field", field);

    // Update all of our sensors.
    leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
    leftEncoderSim.setRate(driveSim.getLeftVelocityMetersPerSecond());
    rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
    rightEncoderSim.setRate(driveSim.getRightVelocityMetersPerSecond());
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = simpleMotorFeedforward.calculate(speeds.leftMetersPerSecond); // Speed of the left side of the robot
    final double rightFeedforward = simpleMotorFeedforward.calculate(speeds.rightMetersPerSecond); // Speed ofo the right side of the robot

    final double leftOutput =
        leftPIDController.calculate(leftEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput =
        rightPIDController.calculate(rightEncoder.getRate(), speeds.rightMetersPerSecond);
    leftMotorGroup.setVoltage(leftOutput + leftFeedforward);
    rightMotorGroup.setVoltage(rightOutput + rightFeedforward);
  }

  // Get Wheel Speed
  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(),rightEncoder.getRate());
  }

  // Drive Functions
  public void arcadeDrive() {
    differentialDrive.arcadeDrive(joystick.getY()*Constants.DRIVE_CONSTANTS.ARCADE_DRIVE_Y_SPEED, joystick.getX()*Constants.DRIVE_CONSTANTS.ARCADE_DRIVE_X_SPEED);
  }

  public void tankDrive(){
   differentialDrive.tankDrive(joystick.getRawAxis(1)*Constants.DRIVE_CONSTANTS.TANK_DRIVE_LEFT_SPEED, -joystick.getRawAxis(5)*Constants.DRIVE_CONSTANTS.TANK_DRIVE_RIGHT_SPEED);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotorGroup.setVoltage(leftVolts);
    rightMotorGroup.setVoltage(rightVolts);
    differentialDrive.feed();
  }

  // Group Speed Setters
  public void setLeftMotorGroupSpeed(double speed) {
    leftMotorGroup.set(speed);
  }

  public void setRightMotorGroupSpeed(double speed) {
    rightMotorGroup.set(speed);
  }

  // Set Max Output Speed
  public void setMaxOutput(double maxOutput) {
    differentialDrive.setMaxOutput(maxOutput);
  }

  // Pose2d and kinematic Getters
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }
  
  // Differential Speeds Getter
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  // Sensors Getters
  public double getLeftEncoderDistance() {
   return leftEncoder.getDistance();
  }

  public double getRightEncoderDistance() {
    return rightEncoder.getDistance();
  }

  public double getAverageEncoderDistance() {
    return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2;
  }

  public double getYaw(){
    return navx.getYaw();
  }

  public double getAngle(){
    return driveSim.getHeading().getDegrees();
  }

  public double getTurnRate() {
    return -navx.getRate();
  }

  // Resetters
  public void resetEncoders(){
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public void resetGyro(){
    navx.reset();
  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    odometry.resetPosition(pose, navx.getRotation2d());
  }   

  public void stop(){
    differentialDrive.stopMotor();
  }

}
