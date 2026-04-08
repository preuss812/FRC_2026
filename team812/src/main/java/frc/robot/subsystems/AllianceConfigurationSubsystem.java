// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.AprilTag;
import frc.robot.Robot;
import frc.robot.RobotContainer;

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
  private static DriveSubsystemSRX m_robotDrive;
  private static PoseEstimatorSubsystem m_poseEstimatorSubsystem;
  private static boolean initialized = false;
  private static Alliance currentAlliance = Alliance.Blue;
  private static Translation2d hubCenter = new Translation2d(0,0);
  private static boolean m_isAutonomous = true;
  private static double m_startLine;
  private static AprilTag towerAprilTag;
  private static AprilTag outpostAprilTag;
  public static final int AUTO_MODE_DO_NOTHING = 0;
  public static final int AUTO_MODE_MOVE_OFF_LINE_AND_STOP = 1;
  public static final int AUTO_MODE_RIGHT_TRENCH_RETURN_TRENCH = 2;
  public static final int AUTO_MODE_RIGHT_BUMP = 3;
  public static final int AUTO_MODE_LEFT_BUMP = 4;
  public static final int AUTO_MODE_LEFT_TRENCH_RETURN_TRENCH = 5;
  public static final int AUTO_MODE_LEFT_TRENCH_RETURN_BUMP = 6;
  public static final int AUTO_MODE_RIGHT_TRENCH_RETURN_BUMP = 7;
  private static boolean m_hubActive = true; // Match starts with active hubs.
  private static boolean m_hubActiveSoon = true; // Match starts with active hubs.
  private static final int BLINK_LED_COUNT = 10;
  private static int m_blinkLEDCounter = 0;
  private static boolean m_blinkLEDState = false;

  
  /** Creates a new AllianceConfigurationSubsystem. */
  public AllianceConfigurationSubsystem(DriveSubsystemSRX robotDrive, PoseEstimatorSubsystem poseEstimatorSubsystem) {
    m_robotDrive = robotDrive;
    m_poseEstimatorSubsystem = poseEstimatorSubsystem;
    // Set up the list of possible autonomous modes.  This is used by the autonomous command to determine which plan to run.
    
    Robot.autoChooser.addOption("Left Bump", AUTO_MODE_LEFT_BUMP);
    Robot.autoChooser.addOption("Left Trench Return Trench", AUTO_MODE_LEFT_TRENCH_RETURN_TRENCH);
    Robot.autoChooser.addOption("Left Trench Return Bump", AUTO_MODE_LEFT_TRENCH_RETURN_BUMP);
    Robot.autoChooser.addOption("Right Bump", AUTO_MODE_RIGHT_BUMP);
    Robot.autoChooser.addOption("Right Trench Return Trench", AUTO_MODE_RIGHT_TRENCH_RETURN_TRENCH);
    Robot.autoChooser.addOption("Right Trench Return Bump", AUTO_MODE_RIGHT_TRENCH_RETURN_BUMP);
    Robot.autoChooser.addOption("Do Nothing", AUTO_MODE_DO_NOTHING);
    Robot.autoChooser.addOption("Move Off Line and Stop", AUTO_MODE_MOVE_OFF_LINE_AND_STOP);

    // Add more commands here and define the plan number above.  Put the commands needed for each auto mode in AutonomousPlans.java.
    SmartDashboard.putData("AutoSelector", Robot.autoChooser);

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
    m_hubActive = isHubActive(0.0); // right now.
    m_hubActiveSoon = isHubActive(5.0); // looking ahead 5 seconds.
    SmartDashboard.putBoolean("HubActive", m_hubActive);
    SmartDashboard.putBoolean("HubActiveSoon", m_hubActiveSoon);

    if (!m_isAutonomous) {
      if (!m_hubActive && m_hubActiveSoon) {
        // Hub is inactive but will be active soon, blink LED to warn driver.
        blinkLED();
      } else if (!m_hubActive && !m_hubActiveSoon) {
        // Hub is inactive and will not be inactive soon, turn off LED.
        RobotContainer.setLED(false); // Hub is inactive, turn off LED.
      } else {
        //Hub is active. Determine if we are ready to shoot and blink if not or solid if ready.
        if (RobotContainer.m_poseEstimatorSubsystem.facingHub(ShooterConstants.rotationTolerance)
          && RobotContainer.m_ShooterSubsystem.readyToShoot()) {
          RobotContainer.setLED(true); // Hub is active, turn on LED.
        } else {
          blinkLED();
        }
      }
    } else {
      // In autonomous, just turn on off the LED.
      RobotContainer.setLED(false);
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
    // Set the hub center location for the current alliance.
    setHubCenter(alliance);
    setOutpostAprilTag(alliance);
    setTowerAprilTag(alliance);

    //if (isAutonomous()) setStartingHeading(alliance); // Problems with debug switching from auto -> teleop -> auto
    setStartLine(alliance);
    currentAlliance = alliance;
    
  }
  /*
   * allianceAdjustedHeading - return the heading adjusted for the current alliance.
   * @param heading - the heading in radians to be adjusted for the current alliance.
   * @return - the heading adjusted for the current alliance.
   */
  public static double allianceAdjustedHeading(double heading) {
    if (Alliance.Blue == currentAlliance) {
      return heading;
    } else {
      return MathUtil.angleModulus(heading + Math.PI);
    }
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
   * hubCenter - return the field locatoin for the current alliance's hub center.
   * This allows for easy access to the hub center location without having to check alliance
   * for vision tracking or autonomous driving.
   * @param - alliance blue or red as the current alliance
   */
  public static void setHubCenter(Alliance alliance) {
    if (alliance == Alliance.Blue) {
      hubCenter = FieldConstants.blueHubCenter;
    } else if (alliance == Alliance.Red) {
      hubCenter = FieldConstants.redHubCenter;
    } else {
      hubCenter = FieldConstants.blueHubCenter;
    }
  }

  /*
   * getHubCenter - return the location of the center of the hub for the current robot alliance
   */
  public static Translation2d getHubCenter() {
    return hubCenter;
  }

  /*
   * robotHeadingToHub() - return the heading from the robot to the hub center for the current alliance.
   * 
   */
  public static double robotHeadingToHub() {
    Pose2d robotPose = m_poseEstimatorSubsystem.getCurrentPose();
    return MathUtil.angleModulus(
        Math.atan2(hubCenter.getY() - robotPose.getY(), hubCenter.getX() - robotPose.getX()));
  }

  /*
   * This allows for easy access to the outpost location without having to check alliance
   * for vision tracking or autonomous driving.
   * @param - alliance blue or red as the current alliance
   */
  public static void setOutpostAprilTag(Alliance alliance) {
    if (alliance == Alliance.Blue) {
      outpostAprilTag = VisionConstants.AprilTag.BLUE_OUTPOST;
    } else if (alliance == Alliance.Red) {
      outpostAprilTag = VisionConstants.AprilTag.RED_OUTPOST;
    } else {
      outpostAprilTag = VisionConstants.AprilTag.BLUE_OUTPOST;
    }
  }

/*
   * This allows for easy access to the tower location without having to check alliance
   * for vision tracking or autonomous driving.
   * @param - alliance blue or red as the current alliance
   */
  public static void setTowerAprilTag(Alliance alliance) {
    if (alliance == Alliance.Blue) {
      towerAprilTag = VisionConstants.AprilTag.BLUE_TOWER;
    } else if (alliance == Alliance.Red) {
      towerAprilTag = VisionConstants.AprilTag.RED_TOWER;
    } else {
      towerAprilTag = VisionConstants.AprilTag.BLUE_TOWER;
    }
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

  /*
   * getOutpostAprilTag - return the april tag associated with the outpost for the current robot alliance
   */
  public static AprilTag getOutpostAprilTag() {
    return outpostAprilTag;
  }

  /*
   * getTowerAprilTag - return the april tag associated with the tower for the current robot alliance
   */
  public static AprilTag getTowerAprilTag() {
    return towerAprilTag;
  }
  
  /*
   * allianceAprilTag - return the april tag associated with the current alliance.
  * @param - the blue april tag id.
  * @return - the april tag associated with the current alliance.
   */
  public int allianceAprilTag(int blueAprilTagId) {
    if (currentAlliance == Alliance.Blue) {
      return blueAprilTagId;
    } else {
      // For the red alliance, the april tag ids are offset by 100.
      return FieldConstants.complementaryAprilTag[blueAprilTagId];
    }
  }

  /**
   * robotFrontFacingHub 
   * @return - the rotation in radians for the robot's front to face the hub in field coordinates.
   */
  public static Rotation2d robotFrontFacingHub() {
      if (!initialized) return new Rotation2d(0.0);
      Rotation2d rotation = new Rotation2d(
        AllianceConfigurationSubsystem.robotHeadingToHub()
    );
    return rotation;
  }

  /**
   * isHubActive - determine if the hub will be active for scoring fuel in "lookAhead" seconds from now.
   * @paraam lookAhead (double) the number of seconds to look ahead.
   * @return (boolean) true if the the hub is active for scoring.
   */
  public boolean isHubActive(double lookAhead) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return false;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime() - lookAhead; // minus because it's a count down timer.
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
    if (gameData.isEmpty()) {
      return true;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return true;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active = switch (alliance.get()) {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
    };

    if (matchTime > 130) {
      // Transition shift, hub is active.
      return true;
    } else if (matchTime > 105) {
      // Shift 1
      return shift1Active;
    } else if (matchTime > 80) {
      // Shift 2
      return !shift1Active;
    } else if (matchTime > 55) {
      // Shift 3
      return shift1Active;
    } else if (matchTime > 30) {
      // Shift 4
      return !shift1Active;
    } else {
      // End game, hub always active.
      return true;
    }
  }

  /**
   * hubActive - return hub status for the current alliance.
   * @return (boolean) true if the hub will is active.
   */
  public static boolean hubActive() {
    return m_hubActive;
  }

  /**
   * hubActive - return hub status for the current alliance.
   * @return (boolean) true if the hub will be active in 3 seconds.
   */
  public static boolean hubActiveSoon() {
    return m_hubActiveSoon;
  }

  private void blinkLED() {
    if (m_blinkLEDCounter <= 0) {
      m_blinkLEDState = !m_blinkLEDState;
      RobotContainer.setLED(m_blinkLEDState);
      m_blinkLEDCounter = BLINK_LED_COUNT;
    } else {
      m_blinkLEDCounter--;
    }
  }
}
