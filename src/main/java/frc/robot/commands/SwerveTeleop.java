// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OI;

public class SwerveTeleop extends CommandBase {
  /** Creates a new SwerveTeleop. */
  Drivetrain drivetrain = Drivetrain.getInstance();
  OI oi;

  public SwerveTeleop(Drivetrain drivetrain, OI oi) {

    this.drivetrain = drivetrain;
    this.oi = oi;

    addRequirements(drivetrain, oi);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    drivetrain.setDrive(
        // applying deadband and setting drive
        MathUtil.applyDeadband(oi.getLeftX(), .1) * Constants.OI_DRIVE_SPEED_RATIO, // flipped (frames of reference)
        -MathUtil.applyDeadband(oi.getLeftY(), .1) * Constants.OI_DRIVE_SPEED_RATIO,
        -MathUtil.applyDeadband(oi.getRightX(), .2) * Constants.OI_TURN_SPEED_RATIO,
        true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
