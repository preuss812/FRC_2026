/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AllianceConfigurationSubsystem;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.DriveSubsystemSRX.DrivingMode;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.commands.AutoSwerveToReefCommand;
import frc.robot.commands.AutoGotoReefCommand;
import frc.robot.commands.DriveWithoutVisionCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

/**
 * Construct the autonomous command.
 * Keep in mind that this class constructs a command.
 * The code below is called NOT during autonomous but before
 * autonomous begins.  As a result conditionals must be lambda functions
 * or must be based on conditions known at the time of Robot.autonomousInit().
 * The trap is to use getPose() or the like in the middle of the plan which will 
 * get the starting location of the robot, not it's location at the time of the reference.
 */
public class Autonomous extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous.
   */
  private final DriveSubsystemSRX m_robotDrive;
  private final PoseEstimatorSubsystem m_PoseEstimatorSubsystem;
  public static int m_autoMode = 1; // Default to move 1 meter and stop;

  /**
   * robotHeadingForCameraToReefCenter - helper function for controlling rotation during autonomous driving.
   * @param x - (double) robot x field coordinate.
   * @param y - (double) robot y field coordinate.
   * @return  - (double) the heading in radians from the robot to the reef center.
   */
  public static double robotHeadingForCameraToReefCenter(Translation2d location) {
    Translation2d reefCenter = AllianceConfigurationSubsystem.getReefCenter();
    return MathUtil.angleModulus(
        Math.atan2(reefCenter.getY() - location.getY(),reefCenter.getX() - location.getX()) + VisionConstants.rearCameraHeading);
  }

  public static double robotHeadingForCameraToPose(Pose2d currentPose, Pose2d targetPose) {
    return MathUtil.angleModulus(
      Math.atan2(targetPose.getY() - currentPose.getY(), targetPose.getX() - currentPose.getX()) + VisionConstants.rearCameraHeading
    );
  }

  public static double robotHeadingForCameraToReefCenter(boolean convertToRed, double x, double y) {
    if (convertToRed) {
      return MathUtil.angleModulus(
        Math.atan2(FieldConstants.redReefCenter.getY() - y, FieldConstants.redReefCenter.getX() - x) + VisionConstants.rearCameraHeading);
    } else {
      return MathUtil.angleModulus(
        Math.atan2(FieldConstants.blueReefCenter.getY() - y, FieldConstants.blueReefCenter.getX() - x) + VisionConstants.rearCameraHeading);
    }
  }

  public static void setAutoMode() {
    try {
      m_autoMode = Robot.autoChooser.getSelected();
    }
    catch(Exception d) {
      m_autoMode = 1;
    }
    if (!(m_autoMode >= 0 && m_autoMode < AutonomousPlans.autoNames.size()))
      m_autoMode = 1;
  }

  // Helper function for instant command.
  public static int getAutoMode() {
    return m_autoMode;
  }

  public Autonomous(RobotContainer robotContainer) {
    
    // get the required subsystems for constructing the plans below.
    m_robotDrive = RobotContainer.m_robotDrive;
    m_PoseEstimatorSubsystem = RobotContainer.m_PoseEstimatorSubsystem;

    // Set up the alliance first.  Other commands need to know which alliance to operate correctly.
    setAutoMode();

    // set up for the current alliance
    addCommands(new InstantCommand(() -> AllianceConfigurationSubsystem.refreshAllianceConfiguration(DriverStation.getAlliance().get())));

    //addCommands(new InstantCommand(() -> setAutoMode()));

    // Initialize the robot before moving.
    addCommands(new SequentialCommandGroup(
      new InstantCommand(() -> AllianceConfigurationSubsystem.setStartingPose(AutonomousPlans.startingPoses.get(getAutoMode()))),
      //new InstantCommand(() -> RobotContainer.m_robotDrive.setAngleDegrees(AutonomousPlans.startingPoses.get(getAutoMode()).getRotation().getDegrees())),
      new InstantCommand(() -> RobotContainer.m_robotDrive.setDrivingMode(DrivingMode.SPEED))
    ));
   
    // For these first 2 modes, just drive 1 meter and wait.
    if (getAutoMode() == AutonomousPlans.AUTO_MODE_ROBOT_DECIDES || getAutoMode() == AutonomousPlans.AUTO_MODE_MOVE_OFF_LINE_AND_STOP) {
      addCommands(new DriveWithoutVisionCommand(m_robotDrive, m_PoseEstimatorSubsystem, null, new Pose2d(-1.0, 0, new Rotation2d(0.0))));
      return;
    }

    // For the do-nothing mode, do not move or take any further action.  
    if (getAutoMode() == AutonomousPlans.AUTO_MODE_DO_NOTHING) {
      return;
    }

    // Perform the initial driving to get from the start line to the reef.
    double timeout = 15;
    if (getAutoMode() == AutonomousPlans.AUTO_MODE_MY_BARGE_TO_OPPOSITE)
      timeout = 12;
    if (getAutoMode() == AutonomousPlans.AUTO_MODE_CENTER_STRAIGHT)
      timeout = 8;
    addCommands(
      new AutoSwerveToReefCommand(m_robotDrive, m_PoseEstimatorSubsystem).withTimeout(timeout), // Should get us to the reef.
      new AutoGotoReefCommand(m_robotDrive,m_PoseEstimatorSubsystem).withTimeout(timeout) // this one makes sure we get to the reef.
    );

    
    
  }
}
