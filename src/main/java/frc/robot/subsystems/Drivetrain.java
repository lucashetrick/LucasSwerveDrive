// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  private static Drivetrain drivetrain;
  AHRS gyro = new AHRS();
  ChassisSpeeds speeds;

  // locations of the modules
  Translation2d leftFrontLocation = new Translation2d(Constants.FEET_TO_METERS * Constants.TRANSLATION_X,
      Constants.FEET_TO_METERS * Constants.TRANSLATION_Y);

  Translation2d rightFrontLocation = new Translation2d(Constants.FEET_TO_METERS * Constants.TRANSLATION_X,
      -Constants.FEET_TO_METERS * Constants.TRANSLATION_Y);

  Translation2d leftBackLocation = new Translation2d(-Constants.FEET_TO_METERS * Constants.TRANSLATION_X,
      Constants.FEET_TO_METERS * Constants.TRANSLATION_Y);

  Translation2d rightBackLocation = new Translation2d(-Constants.FEET_TO_METERS * Constants.TRANSLATION_X,
      -Constants.FEET_TO_METERS * Constants.TRANSLATION_Y);

      
  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      leftFrontLocation, rightFrontLocation, leftBackLocation, rightBackLocation);


  SwerveModule frontLeft = new SwerveModule(Constants.LEFT_FRONT_DRIVE, Constants.LEFT_FRONT_TURN,
      Constants.LEFT_FRONT_CAN_CODER, Constants.FRONT_LEFT_OFFSET);

  SwerveModule frontRight = new SwerveModule(Constants.RIGHT_FRONT_DRIVE, Constants.RIGHT_FRONT_TURN,
      Constants.RIGHT_FRONT_CAN_CODER, Constants.FRONT_RIGHT_OFFSET);

  SwerveModule backLeft = new SwerveModule(Constants.LEFT_BACK_DRIVE, Constants.LEFT_BACK_TURN,
      Constants.LEFT_BACK_CAN_CODER, Constants.BACK_LEFT_OFFSET);

  SwerveModule backRight = new SwerveModule(Constants.RIGHT_BACK_DRIVE, Constants.RIGHT_BACK_TURN,
      Constants.RIGHT_BACK_CAN_CODER, Constants.BACK_RIGHT_OFFSET);

  public Drivetrain() {

  }

  public double getGyroAngle() {

    double angle = gyro.getAngle();

    SmartDashboard.putNumber("gyro angle", angle);

    return angle;
  }

  public void setDrive(double xFeetPerSecond, double yFeetPerSecond, double degreesPerSecond, boolean fieldRelative) {

    if (fieldRelative) {

      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xFeetPerSecond * Constants.FEET_TO_METERS,
          yFeetPerSecond * Constants.FEET_TO_METERS,
          degreesPerSecond * Constants.DEGREES_TO_RADIANS,
          Rotation2d.fromDegrees(getGyroAngle()));

    } else {

      speeds = new ChassisSpeeds(
          xFeetPerSecond * Constants.FEET_TO_METERS,
          yFeetPerSecond * Constants.FEET_TO_METERS,
          degreesPerSecond * Constants.DEGREES_TO_RADIANS);
    }

    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.MAX_MODULE_SPEED); // sets module max speed

    SmartDashboard.putNumber("kinematics output drive ", moduleStates[0].speedMetersPerSecond * Constants.METERS_TO_FEET);
    SmartDashboard.putNumber("kinematics output turn ", moduleStates[0].angle.getDegrees());

    frontLeft.setState(moduleStates[0]);
    frontRight.setState(moduleStates[1]);
    backLeft.setState(moduleStates[2]);
    backRight.setState(moduleStates[3]);
  }

  public static Drivetrain getInstance() {
    if (drivetrain == null) {
      drivetrain = new Drivetrain();
    }
    return drivetrain;
  }
}
