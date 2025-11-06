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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
//import edu.wpi.first.math.trajectory.TrajectoryGenerator;
//import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.utils.TrajectoryGenerator812;

public class FollowTrajectoryPoseCommand extends SequentialCommandGroup {
  public static boolean debug = true;
  // These trajectory config parameters are assumed to be the same for all trajectories.
  // If that is not true, this would need to be moved elsewere or cloned+mutated to provide options.
  public static final TrajectoryConfig config = new TrajectoryConfig(
    AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics);

  /** Creates a new FollowTrajectoryPoseCommand. */
  public FollowTrajectoryPoseCommand(
    DriveSubsystemSRX robotDrive,
    PoseEstimatorSubsystem poseEstimatorSubsystem,
    TrajectoryConfig config,
    Pose2d startingPose,
    List<Translation2d> waypoints,
    Pose2d targetPose) 
  {
    List<Pose2d> waypointPoses = new ArrayList<>();
    ListIterator<Translation2d> waypointIter;
    double startingRotation = startingPose.getRotation().getRadians();
    double endingRotation = targetPose.getRotation().getRadians();
    double deltaRotation = MathUtil.inputModulus(endingRotation - startingRotation, -Math.PI, Math.PI);
    double poseChanges = waypoints.size() + 1;
    double incRotation = deltaRotation / poseChanges; // We will rotate the robot with each successive pose.
    double rotation = startingRotation;
    // Convert the waypoints into poses rotating the robot at each waypoint.
    waypointPoses.add(new Pose2d(startingPose.getTranslation(), new Rotation2d(endingRotation)));
    waypointIter = waypoints.listIterator();
    while (waypointIter.hasNext()) {
      rotation += incRotation;
      Translation2d nextPoint = waypointIter.next();
      waypointPoses.add(new Pose2d(nextPoint, new Rotation2d(endingRotation)));
    }
    waypointPoses.add(targetPose);

    Trajectory trajectory;
    try {
      trajectory = TrajectoryGenerator812.generateTrajectory(          
          waypointPoses,
          config != null ? config : FollowTrajectoryPoseCommand.config); // use default config is none was specified.
    } catch (Exception e) {
        // If the trajectoryGenerator throws an error, 
        // use gotoPose to construct the path.
        ListIterator<Pose2d> poseIter = waypointPoses.listIterator();
        while (poseIter.hasNext()) {
          addCommands(new GotoPoseCommand(robotDrive, poseEstimatorSubsystem, poseIter.next(), false, null)); // For each waypoint.
        }
        SmartDashboard.putString("FT", "catch->gotoPose");
      return;
    }
    if (debug) {
          RobotContainer.m_PoseEstimatorSubsystem.field2d.getObject("trajectory").setTrajectory(trajectory);
    }
    
    var thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      trajectory,
      poseEstimatorSubsystem::getCurrentPose, // Functional interface to feed supplier
      DriveConstants.kDriveKinematics,

      // Position controllers
      new PIDController(AutoConstants.kPXController, 0, 0),
      new PIDController(AutoConstants.kPYController, 0, 0),
      thetaController,
      robotDrive::setModuleStates,
      robotDrive);
    
    addCommands(swerveControllerCommand);
    SmartDashboard.putString("FT", "added");
  }

}
