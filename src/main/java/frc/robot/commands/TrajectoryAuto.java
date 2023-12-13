// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class TrajectoryAuto extends CommandBase {

  Drivetrain drivetrain = Drivetrain.getInstance();
  Pose2d startPose;
  Pose2d endPose;
  List<Translation2d> waypoints;
  TrajectoryConfig config;
  Trajectory trajectory;
  double time; // execute cycle used to calculate elasped time
  PIDController xController;
  PIDController yController;
  ProfiledPIDController rotController;
  HolonomicDriveController controller;


  /**
   * creates a new TrajectoryAuto command with config below
   * @param drivetrain the robot's drivetrain. 
   * @param startPose  the starting pose of the robot (pose2d)
   * @param waypoints  interior waypoints of trajectory (list of translation2ds)
   * @param endPose    the ending pose of the robot (pose2d)
   */
  public TrajectoryAuto(
      Drivetrain drivetrain,
      Pose2d startPose,
      List<Translation2d> waypoints,
      Pose2d endPose) {

    this.drivetrain = drivetrain;
    this.startPose = startPose;
    this.waypoints = waypoints;
    this.endPose = endPose;

    addRequirements(drivetrain);

    xController = new PIDController(1, 0, 0);
    yController = new PIDController(1, 0, 0);
    rotController = new ProfiledPIDController(
      .3, 0, 0, 
      new TrapezoidProfile.Constraints(
        Constants.MAX_TRAJECTORY_SPEED, Constants.MAX_TRAJECTORY_ACCELERATION));

    controller = new HolonomicDriveController(xController, yController, rotController);

    config = new TrajectoryConfig(Constants.MAX_TRAJECTORY_SPEED, Constants.MAX_TRAJECTORY_ACCELERATION);

    trajectory = TrajectoryGenerator.generateTrajectory(
        startPose,
        waypoints,
        endPose,
        config);

    SmartDashboard.putBoolean("trajectory generated", true);
  }


  @Override
  public void initialize() {
  
    time = 0;
    drivetrain.resetOdometry(trajectory.getInitialPose());  // reset the odometry to the starting pose of the trajectory

    // display the trajectory on smartdashboard
    Field2d trajectoryField = new Field2d();
    SmartDashboard.putData(trajectoryField);
    trajectoryField.getObject("Robot").setTrajectory(trajectory);
  }

  @Override
  public void execute() {

    Trajectory.State goal = trajectory.sample(time);
    SmartDashboard.putNumber("time", time);

    ChassisSpeeds adjustedSpeeds = controller.calculate(
        drivetrain.getOdometry(),
        goal,
        goal.poseMeters.getRotation());

    //SmartDashboard.putNumber("time", time)controller.getXController().getPositionError();

    drivetrain.setDrive(
    adjustedSpeeds.vxMetersPerSecond * Constants.METERS_TO_FEET,
    adjustedSpeeds.vyMetersPerSecond * Constants.METERS_TO_FEET,
    adjustedSpeeds.omegaRadiansPerSecond * Constants.RADIANS_TO_DEGREES,
    true);

    time = time + .02;  // add time for cycle
  }

  @Override
  public void end(boolean interrupted) {

    drivetrain.setDrive(0, 0, 0, true);
  }


  @Override
  public boolean isFinished() {

    if (time >= trajectory.getTotalTimeSeconds()) {

      return true;

    } else {

      return false;
    }
  }
}
