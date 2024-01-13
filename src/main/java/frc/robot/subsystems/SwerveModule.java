// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {

  TalonFX driveMotor;
  CANSparkMax turnMotor;
  CANCoder turnEncoder;

  PIDController turnController = new PIDController(Constants.TURN_KP, Constants.TURN_KD, Constants.TURN_KI);

  // Constructor
  public SwerveModule(int driveMotorId, int turnMotorId, int turnEncoderId, double turnEncoderOffset) {

    driveMotor = new TalonFX(driveMotorId, "Canivore_1");
    turnMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);
    turnEncoder = new CANCoder(turnEncoderId, "Canivore_1");

    if (driveMotor != null) {

      SmartDashboard.putNumber("successful created", 1);
    }
    turnEncoder.configMagnetOffset(-turnEncoderOffset); // sets encoder so 0 is forward - negative just makes it work
    turnController.enableContinuousInput(-180, 180); // Pid controller will loop from -180 to 180 continuously
    turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180); // encoder reads -180 to 180

    turnController.setTolerance(2); // sets the tolerance of the turning pid controller.

    // this is setting the pid values of the talon motor controller for velocity mode
    driveMotor.config_kP(0, Constants.DRIVE_KP);
    driveMotor.config_kD(0, Constants.DRIVE_KD);
    driveMotor.config_kI(0, Constants.DRIVE_KI);
    driveMotor.config_kF(0, Constants.DRIVE_KF); // don't know what this is, setting it to 0.
  }

  public void setDriveMotorPercentOutput(double speed) { // sets drive motor in percent output mode (-1.0 to 1.0)

    driveMotor.set(TalonFXControlMode.PercentOutput, speed);
  }

  public void setDriveMotorVelocity(double feetPerSecond) { // sets drive motor in velocity mode (set feet per second)

    double speed = (feetPerSecond * Constants.TICKS_PER_FOOT) / 10.0;
    SmartDashboard.putNumber("velocity mode ticks speed", speed);
    SmartDashboard.putNumber("Velocity error", driveMotor.getClosedLoopError());
    driveMotor.set(TalonFXControlMode.Velocity, speed);
  }

  public void setTurnMotor(double speed) {

    turnMotor.set(speed);
  }

  public double getDriveEncoderPosition() {             // gets drive encoder as distance traveled in feet

    double distance = driveMotor.getSelectedSensorPosition() / Constants.TICKS_PER_FOOT;
    SmartDashboard.putNumber("drive encoder", distance);
    return distance;
  }

  public double getDriveEncoderVelocity() {             // returns drive encoder velocity in feet per second

    double feetPerSecond = (driveMotor.getSelectedSensorVelocity() / Constants.TICKS_PER_FOOT) * 10;
    SmartDashboard.putNumber("drive encoder velocity", feetPerSecond);
    return feetPerSecond;
  }


  public double getTurnEncoder() {                      // gets turn encoder as degrees, -180 180

    double angle = turnEncoder.getAbsolutePosition();   // gets the absoulte position of the encoder. getPosition() returns relative position.
    SmartDashboard.putNumber("turn encoder", angle);
    return angle;
  }

  // public SwerveModuleState getState() {

  //   return new SwerveModuleState(
  //     getDriveEncoderVelocity() * Constants.FEET_TO_METERS, new Rotation2d(getTurnEncoder() * Constants.DEGREES_TO_RADIANS));
  // }


  public SwerveModulePosition getPosition() {

    return new SwerveModulePosition(
      getDriveEncoderPosition() * Constants.FEET_TO_METERS, Rotation2d.fromDegrees(getTurnEncoder()));
  }



  public void setState(SwerveModuleState desiredState) {

    SwerveModuleState optimized = SwerveModuleState.optimize(
      desiredState, 
      new Rotation2d(getTurnEncoder() * Constants.DEGREES_TO_RADIANS));

    SmartDashboard.putNumber("setting drive as ", optimized.speedMetersPerSecond * Constants.METERS_TO_FEET);

    // setDriveMotorVelocity(optimized.speedMetersPerSecond *          
    // Constants.METERS_TO_FEET);

    setDriveMotorVelocity(optimized.speedMetersPerSecond * Constants.METERS_TO_FEET);

    SmartDashboard.putNumber("setting optimized turn setpoint as ", desiredState.angle.getDegrees());

    turnController.setSetpoint(optimized.angle.getDegrees()); // set setpoint

    double speed = -turnController.calculate(getTurnEncoder()); // calculate speed

    boolean atSetpoint = turnController.atSetpoint();

    SmartDashboard.putNumber("turn pid error", turnController.getPositionError());

    SmartDashboard.putNumber("setting turn speed",
        MathUtil.clamp(speed, Constants.TURN_PID_LOW_LIMIT, Constants.TURN_PID_HIGH_LIMIT));

    if (!atSetpoint) {

      setTurnMotor(MathUtil.clamp(speed, Constants.TURN_PID_LOW_LIMIT, Constants.TURN_PID_HIGH_LIMIT));
      // clamp and set speed
    }

  }
}