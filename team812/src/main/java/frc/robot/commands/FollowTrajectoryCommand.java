// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.ListIterator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Utilities;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class FollowTrajectoryCommand extends SequentialCommandGroup {
  public static boolean debug = false;
  // These trajectory config parameters are assumed to be the same for all trajectories.
  // If that is not true, this would need to be moved elsewere or cloned+mutated to provide options.
  public static final TrajectoryConfig config = new TrajectoryConfig(
    AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics);

  /** Creates a new FollowTrajectoryCommand. */
  public FollowTrajectoryCommand(
    DriveSubsystemSRX robotDrive,
    PoseEstimatorSubsystem poseEstimatorSubsystem,
    TrajectoryConfig config,
    List<Pose2d> waypoints
  ) {
    Trajectory trajectory;
    List<Pose2d>  translationWaypoints = new ArrayList<Pose2d>();
    try {
      // set up the rotation for each waypoint to point the robot to the next waypoint.
      for (int i = 0; i < waypoints.size(); i++) {
        Pose2d pose = waypoints.get(i);
  
        if (i == 0) {
          if (waypoints.size() > 1) {
            Pose2d thisPose = waypoints.get(i);
            Pose2d nextPose = waypoints.get(i+1);
            double directionToNext = Math.atan2(nextPose.getTranslation().getY() - thisPose.getTranslation().getY(), nextPose.getTranslation().getX() - thisPose.getTranslation().getX());  
            translationWaypoints.add(new Pose2d(waypoints.get(i).getTranslation(), new Rotation2d(directionToNext))); // Lying about the orientation to get smoother path.
          } else {
            translationWaypoints.add(pose); // There is only one pose, should not happen but keep the orientation.
          }
        } else if (i == waypoints.size() -1) {
          translationWaypoints.add(pose); // For the ending pose, keep the requested ending orientation.
        } else {
          // Not the end, point to the next waypoint.
          Pose2d lastPose = waypoints.get(i-1);
          Pose2d thisPose = waypoints.get(i);
          Pose2d nextPose = waypoints.get(i+1);
          double directionFromLast = Math.atan2(thisPose.getTranslation().getY() - lastPose.getTranslation().getY(), thisPose.getTranslation().getX() - lastPose.getTranslation().getX());  
          double directionToNext = Math.atan2(nextPose.getTranslation().getY() - thisPose.getTranslation().getY(), nextPose.getTranslation().getX() - thisPose.getTranslation().getX());  
          double direction = (MathUtil.angleModulus(directionFromLast) + MathUtil.angleModulus(directionToNext))/2.0;
          direction = MathUtil.angleModulus(directionToNext);
          translationWaypoints.add(new Pose2d(waypoints.get(i).getTranslation(), new Rotation2d(direction)));
        }
        Utilities.toSmartDashboard("TTR", translationWaypoints);

      }
      trajectory = TrajectoryGenerator.generateTrajectory(          
          // Pass through these zero interior waypoints, this should probably be something to make sure we dont crash into other robots.
          translationWaypoints,
          config != null ? config : FollowTrajectoryCommand.config); // use default config is none was specified.
          SmartDashboard.putNumber("PTS", translationWaypoints.size());
          RobotContainer.m_PoseEstimatorSubsystem.field2d.getObject("trajectory").setTrajectory(trajectory);

    } catch (Exception e) {
        ListIterator<Pose2d> iter = translationWaypoints.listIterator();
        while (iter.hasNext()) {
          addCommands(new GotoPoseCommand(robotDrive, poseEstimatorSubsystem, iter.next(), false, null)); // For each waypoint.
        }

        if (debug) SmartDashboard.putString("FT", "catch->gotoPose");
      return;
    }
    //if (true || debug) {
      //RobotContainer.m_PoseEstimatorSubsystem.field2d.getObject("trajectory").setTrajectory(trajectory);
    //}

    var thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PreussSwerveControllerCommand debugSwerveControllerCommand = new PreussSwerveControllerCommand(
      robotDrive,
      poseEstimatorSubsystem,
      trajectory,
      poseEstimatorSubsystem::getCurrentPose, // Functional interface to feed supplier
      // Position controllers
      new PIDController(AutoConstants.kPXController, 0, 0),
      new PIDController(AutoConstants.kPYController, 0, 0),
      thetaController,
      robotDrive::driveFieldRelative,
      robotDrive);
    
    addCommands(debugSwerveControllerCommand);
    if (debug) SmartDashboard.putString("FT", "added");
  }

}
