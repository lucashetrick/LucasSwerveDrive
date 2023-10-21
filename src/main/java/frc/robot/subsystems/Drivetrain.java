// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Drivetrain extends SubsystemBase {

  private static Drivetrain drivetrain;

  TalonFX leftFrontDrive = new TalonFX(Constants.LEFT_FRONT_DRIVE);
  TalonFX rightFrontDrive = new TalonFX(Constants.RIGHT_FRONT_DRIVE);
  TalonFX leftBackDrive = new TalonFX(Constants.LEFT_BACK_DRIVE);
  TalonFX rightBackDrive = new TalonFX(Constants.RIGHT_BACK_DRIVE);

  CANSparkMax leftFrontTurn = new CANSparkMax(Constants.LEFT_FRONT_TURN, MotorType.kBrushless);
  CANSparkMax rightFrontTurn = new CANSparkMax(Constants.RIGHT_FRONT_TURN, MotorType.kBrushless);
  CANSparkMax leftBackTurn = new CANSparkMax(Constants.LEFT_BACK_TURN, MotorType.kBrushless);
  CANSparkMax rightBackTurn = new CANSparkMax(Constants.RIGHT_BACK_TURN, MotorType.kBrushless);



  TalonFX[] driveMotors = {

      leftFrontDrive,   
      rightFrontDrive,
      leftBackDrive,
      rightBackDrive
  };
  CANSparkMax[] turnMotors = {

      leftFrontTurn, 
      rightFrontTurn,
      leftBackTurn,
      rightBackTurn
  };



  int index = 0;
  int arrayLength = driveMotors.length;

  

  /** Creates a new Drivetrain. */
  private Drivetrain() {

  }

  

  public void setDriveMotor (TalonFX driveMotor, double speed) {
    driveMotor.set(TalonFXControlMode.PercentOutput, speed);
  }


  public void setTurnMotor (CANSparkMax turnMotor, double speed) {
    turnMotor.set(speed);
  }



  public double getDriveEncoder (TalonFX driveMotor) {
    return driveMotor.getSelectedSensorPosition() / Constants.TICKS_PER_FOOT;
  }



  public double getTurnEncoder (CANSparkMax turnMotor) {
    return turnMotor.getEncoder().getPosition() / Constants.TICKS_PER_REV;
  }


  

  

  public void switchMotors() {
    if (index > arrayLength - 2) {
      index = -1;
    }

    index++;
    System.out.println("index = " + index);


    if (index != 0) {

      //Turn off the previous module
      setDriveMotor(driveMotors[index-1], 0);
      setTurnMotor(turnMotors[index-1], 0);

    } else if (index == 0) {

      //Turn off the previous module if index=0
      setDriveMotor(driveMotors[arrayLength-1], 0);
      setTurnMotor(turnMotors[arrayLength-1], 0);
    }
  }




  public void swerveTest() {

      setDriveMotor(driveMotors[index], .2);
      setTurnMotor(turnMotors[index], .2);
  }




  public void allAtOnce() {

    for (int i = 0; i<arrayLength; i++) {
      setDriveMotor(driveMotors[i], .2);
    }

    for (int i = 0; i<arrayLength; i++) {
      setTurnMotor(turnMotors[i], .2);
    }

  }




  public static Drivetrain getDrivetrain() {
    if (drivetrain == null) {
      drivetrain = new Drivetrain();
    }
    return drivetrain;
  }
}
