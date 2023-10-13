// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;

public class OI extends SubsystemBase {
  private static OI oi;
  Drivetrain drivetrain = Drivetrain.getDrivetrain();
  /** Creates a new OI. */

  Joystick leftJoystick;
  Joystick rightJoystick;
  JoystickButton switchMotors;


  public OI() {
    rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK);
    switchMotors = new JoystickButton(rightJoystick, Constants.JOYSTICK_BUTTON);
  }

  @Override

  public void periodic() {
  }

  public boolean isSwitchMotors () {

    return switchMotors.getAsBoolean();
  }


  public static OI getOi() {
    if (oi == null) {
      oi = new OI();
    }
    return oi;
  }
}
