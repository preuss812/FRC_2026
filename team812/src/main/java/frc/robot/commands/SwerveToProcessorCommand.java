// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants.AprilTag;
import frc.robot.TrajectoryPlans;
import frc.robot.subsystems.AllianceConfigurationSubsystem;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SwerveToProcessorCommand extends PreussSwerveControllerCommand {

  private AprilTag processorAprilTag;
  private static Rotation2d m_robotRotationToFaceProcessor;
  private Pose2d m_aprilTagPose;
  private final boolean m_faceReef;

  /** Creates a new SwerveToProcessorCommand. */
  public SwerveToProcessorCommand(
    DriveSubsystemSRX robotDrive,
    PoseEstimatorSubsystem poseEstimatorSubsystem,
    boolean faceReef
  ) {
    super (
      robotDrive,
      poseEstimatorSubsystem,
      new Trajectory(),
      poseEstimatorSubsystem::getCurrentPose, // Functional interface to feed supplier

      // Position controllers
      new PIDController(AutoConstants.kPXController, 0, 0),
      new PIDController(AutoConstants.kPYController, 0, 0),
      new ProfiledPIDController(
        AutoConstants.kPThetaController,
        0,
        0,
        new TrapezoidProfile.Constraints(Math.PI, Math.PI)
      ),
      (faceReef)
        ? () -> TrajectoryPlans.reefFacingRotationSupplier(poseEstimatorSubsystem)
        : () -> robotRotationToFaceProcessor(),
      robotDrive::driveFieldRelative,
      robotDrive
    );
    m_faceReef = faceReef;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Trajectory trajectory;
    Pose2d startingPose = getPoseEstimatorSubsystem().getCurrentPose();
  
    processorAprilTag = AllianceConfigurationSubsystem.getProcessorAprilTag();

    List<Pose2d> waypoints = new ArrayList<>();
    m_aprilTagPose = null;
    Pose2d nearTargetPose = null;

    waypoints = TrajectoryPlans.planTrajectory(AllianceConfigurationSubsystem.getProcessorWaypoints(), startingPose);
    m_aprilTagPose = getPoseEstimatorSubsystem().getAprilTagPose(processorAprilTag.id());
    if (m_faceReef)
      super.setRotationSupplier(() -> TrajectoryPlans.reefFacingRotationSupplier(getPoseEstimatorSubsystem()));
    else 
      super.setRotationSupplier(() -> m_aprilTagPose.getRotation().plus(new Rotation2d(Math.PI)));

    nearTargetPose = DriveConstants.robotFrontAtPose(m_aprilTagPose, Units.inchesToMeters(12.0));
    m_robotRotationToFaceProcessor = m_aprilTagPose.getRotation().plus(new Rotation2d(Math.PI));
    waypoints.add(nearTargetPose);
    
    if (waypoints.size() > 0 && !startingPose.equals(nearTargetPose)) {
        List<Pose2d> sanitizedWaypoints = TrajectoryPlans.sanitizeWaypoints(waypoints);
        Pose2d[] waypointArray = new Pose2d[sanitizedWaypoints.size()];
      
        waypointArray = sanitizedWaypoints.toArray(waypointArray);
        trajectory = TrajectoryPlans.createTrajectory(waypointArray, TrajectoryPlans.m_forwardTrajectoryConfig);
    } else {
        trajectory = null;
    }
    super.setTrajectory(trajectory);
    super.initialize();
  
  }

  public static Rotation2d robotRotationToFaceProcessor() {
    return m_robotRotationToFaceProcessor;
  }
}
