// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Motor, Encoder, & Joystick IDS

  // Drive Motors
  public static final int RIGHT_FRONT_DRIVE = 1;
  public static final int RIGHT_BACK_DRIVE = 3;
  public static final int LEFT_FRONT_DRIVE = 5;
  public static final int LEFT_BACK_DRIVE = 7;

  // Turn Motors
  public static final int RIGHT_FRONT_TURN = 12;
  public static final int RIGHT_BACK_TURN = 9;
  public static final int LEFT_FRONT_TURN = 10;
  public static final int LEFT_BACK_TURN = 11;

  // Driving Encoders
  public static final double TALON_GEAR_RATIO = 6.75;
  public static final double WHEEL_DIAMETER = .33333333;
  public static final int TALON_TICKS_PER_MOTOR_REV = 2048;
  public static final double TICKS_PER_FOOT = (TALON_TICKS_PER_MOTOR_REV * TALON_GEAR_RATIO)
      / (WHEEL_DIAMETER * Math.PI);

  // Turning Encoders
  public static final int RIGHT_FRONT_CAN_CODER = 2;
  public static final int RIGHT_BACK_CAN_CODER = 4;
  public static final int LEFT_FRONT_CAN_CODER = 6;
  public static final int LEFT_BACK_CAN_CODER = 8;

  // Joystick Ids
  public static final int LEFT_JOYSTICK = 0;
  public static final int RIGHT_JOYSTICK = 1;


  // Turning Motor PID Constants
  public static final double TURN_KP = .01;
  public static final double TURN_KD = .001;
  public static final double TURN_KI = 0;
  public static final double TURN_PID_LOW_LIMIT = -.8;
  public static final double TURN_PID_HIGH_LIMIT = .8;

  // Velocity Mode PID Constants
  public static final double DRIVE_KP = .12;
  public static final double DRIVE_KD = 0;
  public static final double DRIVE_KI = 0;
  public static final double DRIVE_KF = 0;

  // Module translations
  public static final double TRANSLATION_X = 1.25; // set this
  public static final double TRANSLATION_Y = 1.25; // set this

  // Conversions
  public static final double METERS_TO_FEET = 3.28084;
  public static final double FEET_TO_METERS = 1 / 3.28084;
  public static final double DEGREES_TO_RADIANS = 1 / 57.2958;
  public static final double RADIANS_TO_DEGREES = 57.2958;

  // Can coder offsets
  public static final double FRONT_LEFT_OFFSET = -163.4765625; // set this
  public static final double FRONT_RIGHT_OFFSET = -46.40625; // set this
  public static final double BACK_LEFT_OFFSET = 133.681640625; // set this
  public static final double BACK_RIGHT_OFFSET = 10.01953125; // set this

  // speeds
  public static final double MAX_MODULE_SPEED = 10 * FEET_TO_METERS;

  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
