// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants{

    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(26)), ROBOT_MASS);
    public static final double LOOP_TIME  = 0.02; //s, 20ms + 110ms sprk max velocity lag
    public static final double MAX_SPEED  = Units.feetToMeters(16.5);
    // Maximum speed of the robot in meters per second, used to limit acceleration.

  public static final class AutonConstants{
      //  public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
      //  public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class DrivebaseConstants{
      // Hold time on motor brakes when disabled
      public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants{
    
    // Joystick Deadband
    public static final double DEADBAND        = 0.5;
    public static final double LEFT_Y_DEADBAND = 0.5;
    public static final double RIGHT_X_DEADBAND = 0.5;
    
    public static final double TURN_CONSTANT    = 6;
  }

  // Auto Constants
  public static class AutoConstants{
    
      // Auto Drive PIDF
      public static class AutoDrivePIDF{
          public static final double P = 3; //4.5;
          public static final double I = 0.00;
          public static final double D = 0;
          public static final double I_ZONE = 0;
      }
    
      // Auto Turn PIDF
      public static class AutoTurnPIDF{
          public static final double P = 8;  //13;
          public static final double I = 0.0;
          public static final double D = 0.00;
          public static final double I_ZONE = 0.0;
      }
  }
  
  public static class LimelightConstants{
      public static final String LL1 = "limelight";
      public static final String LL2 = "limelight-two";
  }

  // Subsystems Constants.
    public static class AlgaeConstants{
        // Algae ID
        public static final int Algae_Ctrl_ID = 21;
        public static final int Algae_Roller_ID = 22;

        // Algae Config
        public static final boolean Algae_ctrl_Inverted = true;
        public static final boolean Algae_Roller_Inverted = false;
        public static final double Algae_Ctrl_Longest = 1.3;
        public static final double Algae_Ctrl_Shortest = 0.0;
        public static final double Algae_Ctrl_Middle = 0.12;
    }

    // Climber Constants
    public static class ClimberConstants {
        // Climber ID
        public static final int Left_Motor_ID = 31;
        public static final int RightMotor_ID = 32;

        public static final int Encoder_ID = 33;
        
        // Climber Config
        public static final boolean LeftMotor_Inverted = false;
        public static final boolean RightMotor_Inverted = false;
        public static final double Climb_Angle = 0;

        // Climber PIDF
        public static final double P = 0;
        public static final double I = 0;
        public static final double D = 0;
        public static final double F = 0;
    }

    // Elevator Constants
    public static class ElevatorConstants{
        // Elevator ID
        public static final int LeftMotor_ID = 41;
        public static final int RightMotor_ID = 42;

        public static final int Encoder_ID = 43;

        // Elevator Config
        public static final boolean LeftMotor_Inverted = false;
        public static final boolean RightMotor_Inverted = false;

        public static final double floor = 0;
        public static final double L1 = 0;
        public static final double L2 = 0;
        public static final double L3 = 0;
        public static final double L4 = 0;

        public static final double MAX_ACCEL = 0;
        public static final double MAX_VELOCITY = 0;

        // Elevator PIDF
        public static final double P = 0;
        public static final double I = 0;
        public static final double D = 0;
        public static final double F = 0;
    }

    public static class CoralConstants{
        // Coral ID
        public static final int Coral_Motor_ID = 51;
        public static final int Arm_Left_Motor = 52;
        public static final int Arm_Right_Motor = 53;

        public static final int Coral_Encoder_ID = 56;
        public static final int Arm_Encder = 57;

        // Coral Config
        public static final boolean Coral_Inverted = false;
        public static final boolean Arm_Left_Inverted = false;
        public static final boolean Arm_Right_Inverted = false;
        // Intake
        public static final double Coral_Open = 0;
        public static final double Coral_Close = 0;
        // Arm
        public static final double Arm_Station = 0;
        public static final double Arm_Reef = 0;
    }
}
