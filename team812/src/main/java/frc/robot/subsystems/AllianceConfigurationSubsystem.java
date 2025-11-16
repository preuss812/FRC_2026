// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.imageio.plugins.tiff.TIFFDirectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.TrajectoryPlans;
import frc.robot.Utilities;

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
  private static boolean reefCenterSet = false;

  /** Creates a new AllianceConfigurationSubsystem. */
  public AllianceConfigurationSubsystem(DriveSubsystemSRX robotDrive, PoseEstimatorSubsystem poseEstimatorSubsystem) {
    m_robotDrive = robotDrive;
    m_poseEstimatorSubsystem = poseEstimatorSubsystem;
    // robotDrive and poseEstimatorSubsystem are not requirements.  In this class it is read only.  Saving here to avoid global references.
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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

  public static void refreshAllianceConfiguration(Alliance alliance) {
    // Set the reef center location for the current alliance.
    setReefCenter(alliance);
    setStartingHeading(alliance);
    TrajectoryPlans.buildAutoTrajectories(alliance);
  }

  /*
   * reefCenter - return the field locatoin for the current alliance's reef center.
   * This allows for easy access to the reef center location without having to check alliance
   * for vision tracking or autonomous driving.
   * @param - alliance blue or red as the current alliance
   */
  public static void setReefCenter(Alliance alliance) {
    if (Utilities.isBlueAlliance()) {
      reefCenter = new Translation2d(
        FieldConstants.blueReefCenter.getX(),
        FieldConstants.blueReefCenter.getY()
      );
    } else if (Utilities.isRedAlliance()) {
      reefCenter = new Translation2d(
        FieldConstants.redReefCenter.getX(),
        FieldConstants.redReefCenter.getY()
      );
    }
    reefCenterSet = true;
  }

  /*
   * setStartingHeading - set the robot starting heading based on alliance color.
   * This is used by autonomous to initialize the gyro heading so that the robot
   * drives with the correct heading based on alliance color.
   * @param - alliance blue or red as the current alliance
   */
  public static void setStartingHeading(Alliance alliance) {
    // Assume blue start
    Pose2d startingPose = new Pose2d(
      FieldConstants.xCenter, // Not perfect but close enough.
      FieldConstants.yCenter,
      new Rotation2d(0.0)
    );
      
    // If red, transform the starting pose to the red side of the field.
    if (Alliance.Red == alliance) {
      startingPose = FieldConstants.BlueToRedPose(startingPose);
    }
    
    // Initialize the gyro and drivetrain odometry.
    m_robotDrive.setAngleDegrees(startingPose.getRotation().getDegrees());
    m_robotDrive.resetOdometry(startingPose);
    m_poseEstimatorSubsystem.setCurrentPose(startingPose);
  }
}
