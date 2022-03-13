// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;

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
        public static final int FRONT_LEFT_MOTOR_PIN = 0;
        public static final int FRONT_RIGHT_MOTOR_PIN = 1;
        public static final int REAR_LEFT_MOTOR_PIN = 2;
        public static final int REAR_RIGHT_MOTOR_PIN = 3;

        // For PID Controllers
        public static final int KP = 1;
        public static final int GYRO_KP = 1;
        public static final int KI = 0;
        public static final int KD = 0;

        // For Simple Motor Feedforward
        public static final double KS_VOLTS = 0.0;
        public static final double KV_VOLTS = 0.0;

        // For encoder
        public static final int LEFT_DRIVE_ENCODER_A_CHANNEL = 3;
        public static final int LEFT_DRIVE_ENCODER_B_CHANNEL = 1;
        public static final int RIGHT_DRIVE_ENCODER_A_CHANNEL = 4;
        public static final int RIGHT_DRIVE_ENCODER_B_CHANNEL = 5;
        public static final int kENCODER_RESOLUTION = 4096;

        public static final double kTRACK_WIDTH = 0.381 * 2; // meters
        public static final double kWHEEL_RADIUS = 0.0508; // meters

        public static final Port NAVX_SPI_PORT = SPI.Port.kMXP;

        public static final double TANK_DRIVE_LEFT_SPEED = 0.5;
        public static final double TANK_DRIVE_RIGHT_SPEED = 0.5;
        
        public static final double ARCADE_DRIVE_Y_SPEED = 0.7;
        public static final double ARCADE_DRIVE_X_SPEED = 0.7;  
        
        public static final double K_DRIVE_TICK_2_FEET = 1.0 / 128 * 6 * Math.PI / 12;
    }

    public static final class AUTO_CONSTANTS{
      public static double kMaxSpeedMetersPerSecond = 10;
      public static double kMaxAccelerationMetersPerSecondSquared = 12;
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

        public static final int FRONT_SHOOTER_MOTOR_PIN = 6;
        public static final int REAR_SHOOTER_MOTOR_PIN = 7;

        public static final int FRONT_ENCODER_CHANNEL_A = 0;
        public static final int FRONT_ENCODER_CHANNEL_B = 7;

        public static final int REAR_ENCODER_CHANNEL_A = 2;
        public static final int REAR_ENCODER_CHANNEL_B = 6;

        public static double SHOOTER_SPEED = 0.7;
    }

}
