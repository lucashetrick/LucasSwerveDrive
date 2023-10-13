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

  // Motor Ids
  public static final int LEFT_MODULE_DRIVE_1 = 1;
  public static final int LEFT_MODULE_DRIVE_2 = 3;
  public static final int RIGHT_MODULE_DRIVE_1 = 5;
  public static final int RIGHT_MODULE_DRIVE_2 = 7;

  public static final int LEFT_MODULE_TURN_1 = 9; // 2
  public static final int LEFT_MODULE_TURN_2 = 10; // 4
  public static final int RIGHT_MODULE_TURN_1 = 11; // 6
  public static final int RIGHT_MODULE_TURN_2 = 12; // 8

  // Joystick Ids
  public static final int RIGHT_JOYSTICK = 0;
  public static final int JOYSTICK_BUTTON = 1;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
