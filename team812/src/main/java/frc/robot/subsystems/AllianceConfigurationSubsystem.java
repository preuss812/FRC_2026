// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.AprilTag;
import frc.robot.AutonomousPlans;
import frc.robot.TrajectoryPlans;

/*
 * AllianceConfigurationSubsystem
 * This class will configure the robot based on the alliance color, red or blue.
 * It will handle adjustments to commands the go to alliance specific field locations
 * such as the charging station, scoring locations, etc.
 * It will also handle transformations for trajectories and poses based on alliance color.
 * By confention the field coordinate system is defined from the blue alliance perspective.
 * Therefore, when the robot is on the red alliance, all field relative commands need to 
 * be transformed into the red alliance perspective.
 * This is inteneded to be a singleton class.
 */
public class AllianceConfigurationSubsystem extends SubsystemBase {
  private static  DriveSubsystemSRX m_robotDrive;
  private static PoseEstimatorSubsystem m_poseEstimatorSubsystem;
  private static boolean initialized = false;
  private static Alliance currentAlliance = Alliance.Blue;
  private static Translation2d reefCenter;
  private static AprilTag processorAprilTag;
  private static Pose2d[][] m_processorWaypoints ;
  private static boolean m_isAutonomous = true;
  private static double m_startLine;


  //private static boolean reefCenterSet = false;

  /** Creates a new AllianceConfigurationSubsystem. */
  public AllianceConfigurationSubsystem(DriveSubsystemSRX robotDrive, PoseEstimatorSubsystem poseEstimatorSubsystem) {
    m_robotDrive = robotDrive;
    m_poseEstimatorSubsystem = poseEstimatorSubsystem;
    // robotDrive and poseEstimatorSubsystem are not requirements.  In this class it is read only.  Saving here to avoid global references.
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setAutonomous();

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (!initialized || currentAlliance != alliance.get()) {
        currentAlliance = alliance.get();
        initialized = true;
        /*
         * We are either just starting up or the alliance changed.
         * Reconfigure the robot for the alliance.
         */
        AllianceConfigurationSubsystem.refreshAllianceConfiguration(currentAlliance);
      }
    }
  }

  public static void setAutonomous() {
    m_isAutonomous = DriverStation.isAutonomous();
  }

  public static boolean isAutonomous() {
    return m_isAutonomous;
  }

  public static boolean isBlueAlliance() {
    return currentAlliance == Alliance.Blue;
  }

  public static boolean isRedAlliance() {
    return currentAlliance == Alliance.Red;
  }

  public static void refreshAllianceConfiguration(Alliance alliance) {
    // Set the reef center location for the current alliance.
    setReefCenter(alliance);
    setProcessorAprilTag(alliance);
    //if (isAutonomous()) setStartingHeading(alliance); // Problems with debug switching from auto -> teleop -> auto
    setProcessorWaypoints(alliance);
    setStartLine(alliance);
    AutonomousPlans.buildAutoPlans(alliance);
    
  }

  /*
   * allianceAdjustedMove - return the transform to convert from blue alliance to red alliance movements
   */
  public static Pose2d allianceAdjustedMove(Pose2d pose) {
    // This is set up for a the red half being rotated 180 degrees from the blue half.
    if (Alliance.Blue == currentAlliance) {
      return pose;
    } else {
      return new Pose2d(-pose.getX(), -pose.getY(), pose.getRotation());
    }
  }

  // adjust relative rotation for alliance.
  // For 2025, the field was symmetrical with respect to rotation, so no adjustment is needed.
  // If the field switches to mirroring rotation for red vs blue, this function will need to be updated.
  public static double allianceAdjustedAutonomousRotation(double rotation) {
    return rotation;
  }

  /* 
   * allianceAdjustedTelopRotation - adjust relative rotation for alliance.
   * This adds 180 degrees to the angle if it's the red alliance, otherwise
   * the input value is returned unchanged.
   * @param radians - rotation in radians to be adjusted
   */
  public static double allianceAdjustedTelopRotation(double radians) {
    if (currentAlliance != Alliance.Red) {
      return radians;
     } else {
      return MathUtil.angleModulus(radians + Math.PI);
     }
  }

  public static Rotation2d robotToFieldRotation() {
    if (Alliance.Blue == currentAlliance) {
      return new Rotation2d(0.0);
    } else {
      return new Rotation2d(Math.PI);
    }
  }

  /*
   * reefCenter - return the field locatoin for the current alliance's reef center.
   * This allows for easy access to the reef center location without having to check alliance
   * for vision tracking or autonomous driving.
   * @param - alliance blue or red as the current alliance
   */
  public static void setReefCenter(Alliance alliance) {
    if (alliance == Alliance.Blue) {
      reefCenter = FieldConstants.blueReefCenter;
    } else if (alliance == Alliance.Red) {
      reefCenter = FieldConstants.redReefCenter;
    } else {
      reefCenter = FieldConstants.blueReefCenter;
    }
  }

  /*
   * getReefCenter - return the location of the center of the reef for the current robot alliance
   */
  public static Translation2d getReefCenter() {
    return reefCenter;
  }

  /*
   * setProcessorAprilTag - memorize the april tag associated with the processor for the current alliance.
   * This allows for easy access to the processor location without having to check alliance
   * for vision tracking or autonomous driving.
   * @param - alliance blue or red as the current alliance
   */
  public static void setProcessorAprilTag(Alliance alliance) {
    if (alliance == Alliance.Blue) {
      processorAprilTag = VisionConstants.AprilTag.BLUE_PROCESSOR;
    } else if (alliance == Alliance.Red) {
      processorAprilTag = VisionConstants.AprilTag.RED_PROCESSOR;
    } else {
      processorAprilTag = VisionConstants.AprilTag.BLUE_PROCESSOR;
    }
  }

  /*
   * getProcessorAprilTag - return the april tag associated with the processor for the current robot alliance
   */
  public static AprilTag getProcessorAprilTag() {
    return processorAprilTag;
  }

  public static void setProcessorWaypoints(Alliance alliance) {
    if (alliance != Alliance.Red) {
      m_processorWaypoints = TrajectoryPlans.blueProcessorWaypoints;
    } else {
      m_processorWaypoints = TrajectoryPlans.redProcessorWaypoints;
    }
  }

  public static Pose2d[][] getProcessorWaypoints() {
    return m_processorWaypoints;
  }
  
  public static void setStartLine(Alliance alliance) {
    if (alliance != Alliance.Red) {
      m_startLine = FieldConstants.blueStartLine;
    } else 
      m_startLine = FieldConstants.redStartLine;
    }
  
  public static double getStartLine() {
    return m_startLine;
  }

  /*
   * setStartingHeading - set the robot starting heading based on alliance color.
   * This is used by autonomous to initialize the gyro heading so that the robot
   * drives with the correct heading based on alliance color.
   * @param - alliance blue or red as the current alliance
   */
  public static void setStartingPose(Pose2d startingPose) {
    
    // Initialize the gyro and drivetrain odometry.
    m_robotDrive.setAngleDegrees(startingPose.getRotation().getDegrees());
    m_robotDrive.resetOdometry(startingPose);
    m_poseEstimatorSubsystem.setCurrentPose(startingPose);
  }
}
