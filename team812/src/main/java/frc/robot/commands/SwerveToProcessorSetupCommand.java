// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.AprilTag;
import frc.robot.TrajectoryPlans;
import frc.robot.Utilities;

public class SwerveToProcessorSetupCommand extends Command {
  private final DriveSubsystemSRX robotDrive;
  private final PoseEstimatorSubsystem poseEstimatorSubsystem;
  private final PreussSwerveControllerCommand swerve;
  private AprilTag destination;
  private SequentialCommandGroup commands;
  private Trajectory trajectory;
  private static Rotation2d robotRotationToFaceProcessor;

  
  /** Creates a new SwerveToProcesorComand.
   * This will drive the robot (hopefully) quickly and safely to the processor for the robot's alliance.
   */
  public SwerveToProcessorSetupCommand(
    DriveSubsystemSRX robotDrive,
    PoseEstimatorSubsystem poseEstimatorSubsystem,
    PreussSwerveControllerCommand swerve
  ) {
    this.robotDrive = robotDrive;
    this.poseEstimatorSubsystem = poseEstimatorSubsystem;
    this.swerve = swerve;

    // Use addRequirements() here to declare subsystem dependencies.
    // Next line is commented out because it is causing self cancellation
    // due to the FollowTrajectoryCommand being added to the scheduler.
    //addRequirements(robotDrive, poseEstimatorSubsystem);  // This may be a problem due to self cancellation.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d startingPose = poseEstimatorSubsystem.getCurrentPose();
    int allianceID = Utilities.getAllianceID();
  
    if (allianceID == FieldConstants.BlueAlliance) {
      destination = VisionConstants.AprilTag.BLUE_PROCESSOR;
    } else {
      destination = VisionConstants.AprilTag.RED_PROCESSOR;
    }

    List<Pose2d> waypoints = new ArrayList<>();
    Pose2d aprilTagPose = null;
    Pose2d nearTargetPose = null;
    //Pose2d targetPose = null;
    SmartDashboard.putString("TT","Running");
    //commands = new SequentialCommandGroup();

    if (destination == AprilTag.BLUE_PROCESSOR) {
      waypoints = TrajectoryPlans.planTrajectory(TrajectoryPlans.BlueProcessorPlan, startingPose);
      aprilTagPose = poseEstimatorSubsystem.getAprilTagPose(AprilTag.BLUE_PROCESSOR.id());
      robotRotationToFaceProcessor = aprilTagPose.getRotation().plus(new Rotation2d(Math.PI));
      nearTargetPose = DriveConstants.robotFrontAtPose(aprilTagPose, Units.inchesToMeters(12.0));
      waypoints.add(nearTargetPose);
      //targetPose = Utilities.backToPose(aprilTagPose, 0.5);
    } else if (destination == AprilTag.RED_PROCESSOR) {
      waypoints = TrajectoryPlans.planTrajectory(TrajectoryPlans.RedProcessorPlan, startingPose);
      aprilTagPose = poseEstimatorSubsystem.getAprilTagPose(AprilTag.RED_PROCESSOR.id());
      robotRotationToFaceProcessor = aprilTagPose.getRotation().plus(new Rotation2d(Math.PI));
      nearTargetPose = DriveConstants.robotFrontAtPose(aprilTagPose, Units.inchesToMeters(12.0));
      waypoints.add(nearTargetPose);
      //targetPose = Utilities.backToPose(aprilTagPose, 0.5);
    } else {
      //targetPose = startingPose; // This will end up doing nothing.
      nearTargetPose = startingPose;
    }
    //if (waypoints.size() > 0)
      Utilities.toSmartDashboard("TT Plan", waypoints);
    
    if (waypoints.size() > 0 && !startingPose.equals(nearTargetPose)) {
        List<Pose2d> sanitizedWaypoints = TrajectoryPlans.sanitizeWaypoints(waypoints);
        Pose2d[] waypointArray = new Pose2d[sanitizedWaypoints.size()];
      
        waypointArray = sanitizedWaypoints.toArray(waypointArray);
        trajectory = TrajectoryPlans.createTrajectory(waypointArray, TrajectoryPlans.m_forwardTrajectoryConfig);
    } else {
        trajectory = null;
    }
    swerve.setTrajectory(trajectory);
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

  public static Rotation2d robotRotationToFaceProcessor() {
    return robotRotationToFaceProcessor;
  }
}
