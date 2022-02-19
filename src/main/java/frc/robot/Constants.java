// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

        public static final double TANK_DRIVE_LEFT_SPEED = 0.85;
        public static final double TANK_DRIVE_RIGHT_SPEED = 0.85;
        
        public static final double ARCADE_DRIVE_X_SPEED = 0.7;
        public static final double ARCADE_DRIVE_Z_SPEED = 0.7;       
    }

    public static final class INTAKE_CONSTANTS{
        public static final int INTAKE_MOTOR_PIN = 5;
        public static final double INTAKE_SPEED = 0.7;
    }

    public static final class FEEDER_CONSTANTS{

        public static final int FEEDER_MOTOR_PIN = 6;
        public static final double FEEDER_SPEED = 0.7;
    }

    public static final class SHOOTER_CONSTANTS{

        public static final int SHOOTER_MOTOR_PIN = 7;
        public static double SHOOTER_SPEED = 0.7;
    }
}
