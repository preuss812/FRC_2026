// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Autonomous;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;

/**
 * PointCameraTowardApriltagCommand - rotate robot so that the main camera faces the april tag.
 */
public class PointCameraTowardApriltagCommand extends GotoPoseCommand {

  private final int m_apriltagId;
  private final Pose2d m_apriltagPose;

  /**
 * PointCameraTowardApriltagCommand - rotate robot so that the main camera faces the april tag.
 * @param robotDrive - the drive subsystem
 * @param poseEstimatorSubsystem - the pose estimator.
 * @param apriltagID - the april tag you want to the camera to point at.
 */
public PointCameraTowardApriltagCommand(
    DriveSubsystemSRX robotDrive
    , PoseEstimatorSubsystem poseEstimatorSubsystem
    , int aprilTagID
    ) {
    super(robotDrive, poseEstimatorSubsystem, new Pose2d(), true, null );
    this.m_apriltagId = aprilTagID;
    this.m_apriltagPose = poseEstimatorSubsystem.getAprilTagPose(m_apriltagId);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      // Calculate the pose to face 
    Pose2d currentPose = getPoseEstimatorSubsystem().getCurrentPose();
    setTargetPose(new Pose2d(
      currentPose.getTranslation(),
      new Rotation2d(Autonomous.robotHeadingForCameraToPose(currentPose, m_apriltagPose))
    ));
    super.initialize();
  }

  
}
