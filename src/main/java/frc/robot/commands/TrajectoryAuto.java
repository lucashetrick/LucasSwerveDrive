// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class TrajectoryAuto extends CommandBase {
  
  Drivetrain drivetrain = Drivetrain.getInstance();
  Pose2d startPose;
  Pose2d endPose;
  List<Translation2d> waypoints;
  boolean reversed;
  TrajectoryConfig config;
  Trajectory trajectory;
  int i;

  HolonomicDriveController controller = new HolonomicDriveController(
  new PIDController(.5, 0, 0), new PIDController(.5, 0, 0),
  new ProfiledPIDController(.5, 0, 0,
    new TrapezoidProfile.Constraints(6.28, 3.14)));


  
/**
 Constructs a new TrajectoryAuto Command with DEFAULT config
 * @param drivetrain the robot's drivetrain.
 * @param startPose the starting pose of the robot (pose2d)
 * @param waypoints interior waypoints of trajectory (arraylist of translation2ds)
 * @param endPose the ending pose of the robot (pose2d)
 * @param reversed if the trajectory should be reversed or not.
 */
  public TrajectoryAuto(
    Drivetrain drivetrain, 
    Pose2d startPose, 
    List<Translation2d> waypoints, 
    Pose2d endPose, 
    boolean reversed) {

    this.drivetrain = drivetrain;
    this.startPose = startPose;
    this.waypoints = waypoints;
    this.endPose = endPose;
    this.reversed = reversed;
    
    addRequirements(drivetrain);
    }


/**
 Constructs a new TrajectoryAuto Command with CUSTOM config
 * @param drivetrain the robot's drivetrain.
 * @param startPose the starting pose of the robot (pose2d)
 * @param waypoints interior waypoints of trajectory (arraylist of translation2ds)
 * @param endPose the ending pose of the robot (pose2d)
 * @param reversed if the trajectory should be reversed or not.
 * @param config the configuration of the trajectory (max speed and acceleration)
 */
  public TrajectoryAuto(
    Drivetrain drivetrain, 
    Pose2d startPose, 
    List<Translation2d> waypoints, 
    Pose2d endPose, 
    boolean reversed, 
    TrajectoryConfig config) {

    this.drivetrain = drivetrain;
    this.startPose = startPose;
    this.waypoints = waypoints;
    this.endPose = endPose;
    this.reversed = reversed;
    this.config = config;
    
    addRequirements(drivetrain);
  }

  
  @Override
  public void initialize() {

    if (config == null) {
      config = new TrajectoryConfig(Constants.MAX_TRAJECTORY_SPEED, Constants.MAX_TRAJECTORY_ACCELERATION);
    }

    config.setReversed(reversed);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        startPose,
        waypoints,
        endPose,
        config);

    drivetrain.resetOdometry(trajectory.getInitialPose()); // reset the odometry
  }


  @Override
  public void execute() {
  
  // Trajectory.State goal = trajectory.sample(i / 50);
  // ChassisSpeeds adjustedSpeeds = controller.calculate(endPose, goal, null);

  // drivetrain.setDrive(
  //   adjustedSpeeds.vxMetersPerSecond, 
  //   adjustedSpeeds.vyMetersPerSecond, 
  //   adjustedSpeeds.omegaRadiansPerSecond, 
  //   true);
  // i++;
  }
  

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
