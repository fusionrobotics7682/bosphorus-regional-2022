// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Example GitHub Commit 

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DRIVE_CONSTANTS{
        public static final int FRONT_LEFT_MOTOR_PIN = 1;
        public static final int FRONT_RIGHT_MOTOR_PIN = 2;
        public static final int REAR_LEFT_MOTOR_PIN = 3;
        public static final int REAR_RIGHT_MOTOR_PIN = 4;

        public static final int KP = 1;
        public static final int KI = 0;
        public static final int KD = 0;

        public static final int LEFT_DRIVE_ENCODER_A_CHANNEL = 9;
        public static final int LEFT_DRIVE_ENCODER_B_CHANNEL = 8;
        public static final int RIGHT_DRIVE_ENCODER_A_CHANNEL = 0;
        public static final int RIGHT_DRIVE_ENCODER_B_CHANNEL = 1;

        public static double tankDriveLeftMotorsSpeed = 0.85;
        public static double tankDriveRightMotorsSpeed = 0.85;
        
        public static double arcadeDriveXSpeed = 0.7;
        public static double arcadeDriveZSpeed = 0.7;

        public static double getTankDriveLeftMotorsSpeed(){
            return tankDriveLeftMotorsSpeed;
        }

        public static void setTankDriveLeftMotorsSpeed(double speed){
            tankDriveLeftMotorsSpeed = speed;
        }

        public static double getTankDriveRightMotorsSpeed(){
            return tankDriveRightMotorsSpeed;
        }

        public static void setTankDriveRightMotorsSpeed(double speed){
            tankDriveRightMotorsSpeed = speed;
        }

        public static double getArcadeDriveXSpeed(){
            return arcadeDriveXSpeed;
        }

        public static void setArcadeDriveXSpeed(double speed){
            arcadeDriveXSpeed = speed;
        }

        public static double getArcadeDriveZSpeed(){
            return arcadeDriveZSpeed;
        }

        public static void setArcadeDriveZSpeed(double speed){
            arcadeDriveZSpeed = speed;
        }
       
    }

    public static final class INTAKE_CONSTANTS{
        public static final int INTAKE_MOTOR_PIN = 5;
        public static double intakeSpeed = 0.7;

        public static double getIntakeSpeed(){
            return intakeSpeed;
        }

        public static void setIntakeSpeed(double speed){
            intakeSpeed = speed;
        }
    }

    public static final class FEEDER_CONSTANTS{
        public static final int FEEDER_MOTOR_PIN = 5;
        public static double feederSpeed = 0.7;

        public static double getFeederSpeed(){
            return feederSpeed;
        }

        public static void setFeederSpeed(double speed){
            feederSpeed = speed;
        }
    }

    public static final class SHOOTER_CONSTATS{
        public static final int SHOOTER_MOTOR_PIN = 7;
        public static double shooterSpeed = 0.7;

        public static double getShooterSpeed(){
            return shooterSpeed;
        }

        public static void setShooterSpeed(double speed){
            shooterSpeed = speed;
        }
    }
}
