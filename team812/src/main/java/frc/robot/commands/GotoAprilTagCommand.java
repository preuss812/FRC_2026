// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.utils.DrivingConfig;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;

/**
 * GotoAprilTagCommand - pick the best visible april tag and drive to it.
 */
public class GotoAprilTagCommand extends GotoPoseCommand {
  
  /** Creates a new command to move the robot to the specified pose. */
  
  private final double m_targetDistance;
  private Pose2d m_aprilTagPose;
  private boolean m_simulatingRobot = RobotContainer.isSimulation(); // force robot starting position and april tag number for debugging purposes.
  private int m_simulationNumber = 0;
  private int[] m_simulationAprilTagIDs = new int [] {
    6,6,6,
    7,7,7,
    19,19,19,
    17,17,17
  };
  private Pose2d[] m_simulatedRobotStartingPoses = new Pose2d[] {
  new Pose2d(12.5, 0.60, new Rotation2d(Units.degreesToRadians(-60))),
  new Pose2d(15.5, 0.60, new Rotation2d(Units.degreesToRadians(-60))),
  new Pose2d(15.5, 2.60, new Rotation2d(Units.degreesToRadians(-60))),

  new Pose2d(14.5, 7.60, new Rotation2d(Units.degreesToRadians(0))),
  new Pose2d(15.5, 5.60, new Rotation2d(Units.degreesToRadians(10))),
  new Pose2d(15.5, 2.60, new Rotation2d(Units.degreesToRadians(-10))),
  
  new Pose2d(3.5, 7.60, new Rotation2d(Units.degreesToRadians(110))),
  new Pose2d(1.5, 7.60, new Rotation2d(Units.degreesToRadians(130))),
  new Pose2d(1.5, 5.60, new Rotation2d(Units.degreesToRadians(130))),

  new Pose2d(1.5, 2.60, new Rotation2d(Units.degreesToRadians(-125))),
  new Pose2d(1.5, 0.60, new Rotation2d(Units.degreesToRadians(-130))),
  new Pose2d(4.5, 0.60, new Rotation2d(Units.degreesToRadians(-120)))
  };

  /**
   * Drive to the specified distance from the best april tag currently in view.
   * @params robotDrive - the DriveSubsystemSRX drivetrain to drive the robot
   * @params poseEstimatorSubsystem - the pose estimator subsystem
   * @params targetDistance - the distance in meters to be away from the April Tag.
   * @params config - the driving configuration or null for default config.
   */
  public GotoAprilTagCommand(
    DriveSubsystemSRX robotDrive
    , PoseEstimatorSubsystem poseEstimatorSubsystem
    , double targetDistance
    , DrivingConfig config
    ) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    super(robotDrive, poseEstimatorSubsystem, new Pose2d(0,0,new Rotation2d(0.0)), true, config);
    
    this.m_targetDistance = targetDistance;
    m_simulatingRobot = RobotContainer.isSimulation();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    // If se see an april tag, compute the pose for the camera to be 'targetDistance' away
    // on the projection line from the tag.
    int fiducialId = VisionConstants.NO_TAG_FOUND;
    if (m_simulatingRobot) {
      // For simulation, use the pre-defined staring poses and tags.
      getPoseEstimatorSubsystem().setCurrentPose(m_simulatedRobotStartingPoses[m_simulationNumber]);
      fiducialId = m_simulationAprilTagIDs[m_simulationNumber++];
      if (m_simulationNumber >= m_simulationAprilTagIDs.length) m_simulationNumber = 0;
    } else {
      fiducialId = getPoseEstimatorSubsystem().lastAprilTagSeen();
    }
    
    if (fiducialId >= 0) {
      m_aprilTagPose = getPoseEstimatorSubsystem().getAprilTagPose(fiducialId);
      setTargetPose(DriveConstants.robotRearAtPose(m_aprilTagPose, m_targetDistance));
    } else {
      setTargetPose(getPoseEstimatorSubsystem().getCurrentPose()); // effectively do nothing.
    }
    super.initialize();
  }

} // GotoAprilTagCommand class
