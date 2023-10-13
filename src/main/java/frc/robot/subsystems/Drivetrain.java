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

  TalonFX leftMotorDrive1 = new TalonFX(Constants.LEFT_MODULE_DRIVE_1);
  TalonFX leftMotorDrive2 = new TalonFX(Constants.LEFT_MODULE_DRIVE_2);
  TalonFX rightMotorDrive1 = new TalonFX(Constants.RIGHT_MODULE_DRIVE_1);
  TalonFX rightMotorDrive2 = new TalonFX(Constants.RIGHT_MODULE_DRIVE_2);

  CANSparkMax leftTurnMotor1 = new CANSparkMax(Constants.LEFT_MODULE_TURN_1, MotorType.kBrushless);
  CANSparkMax leftTurnMotor2 = new CANSparkMax(Constants.LEFT_MODULE_DRIVE_2, MotorType.kBrushless);
  CANSparkMax rightTurnMotor1 = new CANSparkMax(Constants.RIGHT_MODULE_TURN_1, MotorType.kBrushless);
  CANSparkMax rightTurnMotor2 = new CANSparkMax(Constants.RIGHT_MODULE_TURN_2, MotorType.kBrushless);

  TalonFX[] driveMotors = {
      leftMotorDrive1,
      leftMotorDrive2,
      rightMotorDrive1,
      rightMotorDrive2
  };
  CANSparkMax[] turnMotors = {
      leftTurnMotor1,
      leftTurnMotor2,
      rightTurnMotor1,
      rightTurnMotor2
  };

  int index = 0;
  int arrayLength = driveMotors.length-1;


  /** Creates a new Drivebase. */
  private Drivetrain() {
  }

  

  public void switchMotors() {

    
    
    if (index > arrayLength-1) {
      System.out.println(arrayLength);
      index = 0;
    }
    

    index++;

    System.out.println("index = " + index);
  }


  public void setDrive() {

    if (index != 0) {
      driveMotors[index-1].set(TalonFXControlMode.PercentOutput, 0);
      turnMotors[index-1].set(0);
    }

    System.out.println(index);

    driveMotors[index].set(TalonFXControlMode.PercentOutput, .2);

    turnMotors[index].set(.2);
  }



  // public void setDrive(TalonFX driveModule, double driveMotorSpeed, 
  //                     CANSparkMax turnModule, double turnMotorSpeed) {


  //   driveModule.set(TalonFXControlMode.PercentOutput, driveMotorSpeed);
  //   turnModule.set(turnMotorSpeed);
  // }

  public static Drivetrain getDrivetrain() {
    if (drivetrain == null) {
      drivetrain = new Drivetrain();
    }
    return drivetrain;
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }
}
