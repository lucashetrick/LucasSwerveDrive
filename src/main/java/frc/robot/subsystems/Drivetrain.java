// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  private static Drivetrain drivetrain;
  AHRS gyro = new AHRS(I2C.Port.kOnboard);
  private final Field2d odometryFieldPos = new Field2d();
  ChassisSpeeds speeds;
  Pose2d pose;

  // locations of the modules
  Translation2d leftFrontLocation = new Translation2d(Constants.FEET_TO_METERS * Constants.TRANSLATION_X,
      Constants.FEET_TO_METERS * Constants.TRANSLATION_Y);

  Translation2d rightFrontLocation = new Translation2d(Constants.FEET_TO_METERS * Constants.TRANSLATION_X,
      -Constants.FEET_TO_METERS * Constants.TRANSLATION_Y);

  Translation2d leftBackLocation = new Translation2d(-Constants.FEET_TO_METERS * Constants.TRANSLATION_X,
      Constants.FEET_TO_METERS * Constants.TRANSLATION_Y);

  Translation2d rightBackLocation = new Translation2d(-Constants.FEET_TO_METERS * Constants.TRANSLATION_X,
      -Constants.FEET_TO_METERS * Constants.TRANSLATION_Y);


  SwerveModule frontLeft = new SwerveModule(Constants.LEFT_FRONT_DRIVE, Constants.LEFT_FRONT_TURN,
      Constants.LEFT_FRONT_CAN_CODER, Constants.FRONT_LEFT_OFFSET);

  SwerveModule frontRight = new SwerveModule(Constants.RIGHT_FRONT_DRIVE, Constants.RIGHT_FRONT_TURN,
      Constants.RIGHT_FRONT_CAN_CODER, Constants.FRONT_RIGHT_OFFSET);

  SwerveModule backLeft = new SwerveModule(Constants.LEFT_BACK_DRIVE, Constants.LEFT_BACK_TURN,
      Constants.LEFT_BACK_CAN_CODER, Constants.BACK_LEFT_OFFSET);

  SwerveModule backRight = new SwerveModule(Constants.RIGHT_BACK_DRIVE, Constants.RIGHT_BACK_TURN,
      Constants.RIGHT_BACK_CAN_CODER, Constants.BACK_RIGHT_OFFSET);


  SwerveDriveKinematics kinematics = new SwerveDriveKinematics( // creating kinematics
      leftFrontLocation, rightFrontLocation, leftBackLocation, rightBackLocation);


  SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      kinematics,
      Rotation2d.fromDegrees(getGyroAngle()), // creating odometry
      new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
      }); 



  public Drivetrain() {

    gyro.calibrate();

    while (gyro.isCalibrating()) {
      SmartDashboard.putBoolean("is calibrated", false);
    }

    SmartDashboard.putBoolean("is calibrated", true);

    gyro.reset();


    resetOdometry(new Pose2d((Constants.FIELD_Y_LENGTH-1.895833333) * Constants.FEET_TO_METERS, 
    (Constants.FIELD_X_LENGTH-1.895833333) * Constants.FEET_TO_METERS,
    Rotation2d.fromDegrees(getGyroAngle())));

    SmartDashboard.putData("Field Pos", odometryFieldPos);
  }



  public double getGyroAngle() {

    double angle = gyro.getAngle();
    SmartDashboard.putNumber("gyro angle", angle);
    return angle;
  }



  public void resetOdometry(Pose2d position) {

    odometry.resetPosition(
      Rotation2d.fromDegrees(getGyroAngle()), 
      new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()}, 
      position);

    SmartDashboard.putBoolean("odometry reset pos", true);
  }


  public Pose2d getOdometry() {

    return odometry.getPoseMeters();
  }

  
  public void setDrive(double xFeetPerSecond, double yFeetPerSecond, double degreesPerSecond, boolean fieldRelative) {

    if (fieldRelative) {

      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xFeetPerSecond * Constants.FEET_TO_METERS,
          yFeetPerSecond * Constants.FEET_TO_METERS,
          degreesPerSecond * Constants.DEGREES_TO_RADIANS,
          Rotation2d.fromDegrees(-getGyroAngle())); // negative because gyro reads differently than wpilib

    } else {

      speeds = new ChassisSpeeds(
          xFeetPerSecond * Constants.FEET_TO_METERS,
          yFeetPerSecond * Constants.FEET_TO_METERS,
          degreesPerSecond * Constants.DEGREES_TO_RADIANS);
    }

    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.MAX_MODULE_SPEED); // sets module max speed

    setModuleStates(moduleStates);
  }


  public void setModuleStates(SwerveModuleState[] states) {

    frontLeft.setState(states[0]);
    frontRight.setState(states[1]);
    backLeft.setState(states[2]);
    backRight.setState(states[3]);
  }


  @Override
  public void periodic() {
    
    pose = odometry.update(Rotation2d.fromDegrees(getGyroAngle()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });

    odometryFieldPos.setRobotPose(odometry.getPoseMeters());
  }


  
  public static Drivetrain getInstance() {
    if (drivetrain == null) {
      drivetrain = new Drivetrain();
    }
    return drivetrain;
  }
}
