// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class SwerveConstants{
  public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.00); //originally 4 in template
    public static final double DRIVE_MOTOR_GEAR_RATIO = 6.75;
    public static final double TURN_MOTOR_GEAR_RATIO = 150.0/7;
    public static final double DRIVE_ENCODER_ROT2METERS = DRIVE_MOTOR_GEAR_RATIO*Math.PI*WHEEL_DIAMETER;
    public static final double TURNING_ENCODER_ROT2RAD = TURN_MOTOR_GEAR_RATIO*2*Math.PI;
    public static final double DRIVE_ENCODER_ROT2METERPERSEC = DRIVE_ENCODER_ROT2METERS/60;
    public static final double TURNING_ENCODER_ROT2RADPERSEC = TURNING_ENCODER_ROT2RAD/60;
    public static final double KP_TURNING = 0.575;

    public static final double DRIVETRAIN_MAX_SPEED = 5.3;//4.0, 5.5;
    public static final double DRIVETRAIN_MAX_ANGULAR_SPEED = 5 * Math.PI; //3.5, 4.25, 5

    //Teleop constraints
    public static final double TELE_DRIVE_MAX_SPEED = DRIVETRAIN_MAX_SPEED / 0.85;
    public static final double TELE_DRIVE_MAX_ANGULAR_SPEED = DRIVETRAIN_MAX_ANGULAR_SPEED / 1.75;
    public static final double TELE_DRIVE_MAX_ACCELERATION = 7.5; //3
    public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION = 15; //

    public static final double AUTO_KP_TTANSLATION = 1.35; //1.15
    public static final double AUTO_KP_ROTATIONAL = 0.1; //0.1

    public static final double TRACK_WIDTH = Units.inchesToMeters(23.875);
    public static final double WHEEL_BASE = Units.inchesToMeters(23.875);
    public static final double DRIVE_BASE_RADIUS = Math.sqrt((Math.pow(TRACK_WIDTH, 2) + Math.pow(WHEEL_BASE, 2))) / 2.0;

    public static final int LEFT_FRONT_DRIVE_ID = 7;
     public static final int RIGHT_FRONT_DRIVE_ID = 1;
     public static final int LEFT_BACK_DRIVE_ID = 5; // 5
     public static final int RIGHT_BACK_DRIVE_ID = 3; // 3
    
     public static final int LEFT_FRONT_TURN_ID = 6;
     public static final int RIGHT_FRONT_TURN_ID = 8;
     public static  final int LEFT_BACK_TURN_ID = 4; // 4 
     public static final int RIGHT_BACK_TURN_ID = 2; // 2
    
    public static final int LEFT_FRONT_CANCODER_ID = 3;
    public static final int RIGHT_FRONT_CANCODER_ID = 4;
    public static final int LEFT_BACK_CANCODER_ID = 0; // 2
    public static final int RIGHT_BACK_CANCODER_ID = 1;

    public static final int PIGEON_ID = 0;

    public static final boolean LEFT_FRONT_DRIVE_MOTOR_REVERSED = false;
    public static final boolean LEFT_FRONT_TURN_MOTOR_REVERSED = true;
    
    public static final boolean RIGHT_FRONT_DRIVE_MOTOR_REVERSED = false;
    public static final boolean RIGHT_FRONT_TURN_MOTOR_REVERSED = true;

    public static final boolean LEFT_BACK_DRIVE_MOTOR_REVERSED = true;
    public static final boolean LEFT_BACK_TURN_MOTOR_REVERSED = true;

    public static final boolean RIGHT_BACK_DRIVE_MOTOR_REVERSED = false;
    public static final boolean RIGHT_BACK_TURN_MOTOR_REVERSED = true;

    public static final boolean LEFT_FRONT_CANCODER_REVERSED = false;
    public static final boolean RIGHT_FRONT_CANCODER_REVERSED = false;
    public static final boolean LEFT_BACK_CANCODER_REVERSED = false;
    public static final boolean RIGHT_BACK_CANCODER_REVERSED = false;

    public static  double LEFT_FRONT_OFFSET =0.205811;//0.206055; //0.222412;//0.213135;//0.216064;//0.721680;//0.732666;//0.292969;//0.723145;//0.717041;//0.717041;//0.710938 ;//0.725098;//0.719971;	
    public static  double RIGHT_FRONT_OFFSET = 0.798828;//0.301514;//0.799072;//0.302246;//0.306885;//0.792725;//0.712891;//0.817383;
    public static  double LEFT_BACK_OFFSET = 0.5217290;//0.524902;//0.017334;//0.794189;//0.818115;//0.300781;//0.303711;//0.847412;//0.448730;//0.455322;//0.418945;//0.490967;//0.519043;
    public static  double RIGHT_BACK_OFFSET = 0.666748;//0.514160;//0.512451;//0.011475 ;//0.012451;//0.014404;//0.510498;//0.504883;//0.013916; 

    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
    );

  }
public static class DeadBandConstants{
  public static double xDeadband = 0.05;
  public static double yDeadband = 0.05;
  public static double turnDeadband = 0.05;

}
}
