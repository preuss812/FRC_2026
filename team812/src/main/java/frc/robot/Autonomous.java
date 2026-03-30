/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.autoCommands.LeftBumpCommand;
import frc.robot.autoCommands.LeftTrenchCommand;
import frc.robot.autoCommands.RightBumpCommand;
import frc.robot.autoCommands.RightTrenchCommand;
import frc.robot.commands.DriveWithoutVisionCommand;
import frc.robot.subsystems.AllianceConfigurationSubsystem;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.DriveSubsystemSRX.DrivingMode;
import frc.robot.subsystems.PoseEstimatorSubsystem;

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
  
 // SmartDashboard.putNumber("Auto Delay", 0.0);

  /**
   * robotHeadingForCameraToHubCenter - helper function for controlling rotation during autonomous driving.
   * @param x - (double) robot x field coordinate.
   * @param y - (double) robot y field coordinate.
   * @return  - (double) the heading in radians from the robot to the hub center.
   */
  public static double robotHeadingForCameraToHubCenter(Translation2d location) {
    Translation2d hubCenter = AllianceConfigurationSubsystem.getHubCenter();
    return MathUtil.angleModulus(
        Math.atan2(hubCenter.getY() - location.getY(),hubCenter.getX() - location.getX()) + VisionConstants.aprilCameraHeading);
  }

  public static double robotHeadingForCameraToPose(Pose2d currentPose, Pose2d targetPose) {
    return MathUtil.angleModulus(
      Math.atan2(targetPose.getY() - currentPose.getY(), targetPose.getX() - currentPose.getX()) + VisionConstants.aprilCameraHeading
    );
  }

  public static double robotHeadingForCameraToHubCenter(boolean convertToRed, double x, double y) {
    if (convertToRed) {
      return MathUtil.angleModulus(
        Math.atan2(FieldConstants.redHubCenter.getY() - y, FieldConstants.redHubCenter.getX() - x) + VisionConstants.aprilCameraHeading);
    } else {
      return MathUtil.angleModulus(
        Math.atan2(FieldConstants.blueHubCenter.getY() - y, FieldConstants.blueHubCenter.getX() - x) + VisionConstants.aprilCameraHeading);
    }
  }

  // setAutoMode - read the auto mode from network tables and memorize the result.
  public static void setAutoMode() {
    try {
      m_autoMode = Robot.autoChooser.getSelected();
    }
    catch(Exception d) {
      m_autoMode = AllianceConfigurationSubsystem.AUTO_MODE_DO_NOTHING;
    }
    
  }

  // Helper function for instant command.
  public static int getAutoMode() {
    return m_autoMode;
  }

  public Autonomous(RobotContainer robotContainer) {
    
    // get the required subsystems for constructing the plans below.
    m_robotDrive = RobotContainer.m_robotDrive;
    m_PoseEstimatorSubsystem = RobotContainer.m_poseEstimatorSubsystem;
    AllianceConfigurationSubsystem.refreshAllianceConfiguration( DriverStation.getAlliance().get());

    // Set up the alliance first.  Other commands need to know which alliance to operate correctly.
    setAutoMode();

    switch(m_autoMode) {
      
      case AllianceConfigurationSubsystem.AUTO_MODE_MOVE_OFF_LINE_AND_STOP:
        // Tell the robot it is on the start line in the center of the field facing toward field center.
        // The robot could be placed anywhere on the start line.  Seeing an apriltag will 'cure' the unknown location.
        addCommands(
          new SequentialCommandGroup(
            new InstantCommand(
              () -> AllianceConfigurationSubsystem.setStartingPose(
                new Pose2d(
                  AllianceConfigurationSubsystem.getStartLine(), 
                  FieldConstants.yCenter, 
                  new Rotation2d(AllianceConfigurationSubsystem.allianceAdjustedHeading(0.0)) // facing toward field center.
                )
              )
            ),
            new InstantCommand(() -> RobotContainer.m_robotDrive.setDrivingMode(DrivingMode.SPEED))
          )
        );
        // Drive one meter backwards.  That should get us off the start line.
        addCommands(new DriveWithoutVisionCommand(m_robotDrive, m_PoseEstimatorSubsystem,  new Pose2d(-1.0, 0, new Rotation2d(0.0)), null));
        break;
      
      case AllianceConfigurationSubsystem.AUTO_MODE_RIGHT_TRENCH_RETURN_TRENCH:
        addCommands(new RightTrenchCommand(Robot.m_rightTrenchGather, Robot.m_rightTrenchReturn, Robot.m_rightTrenchGather2));
        break;
      case AllianceConfigurationSubsystem.AUTO_MODE_RIGHT_TRENCH_RETURN_BUMP:
        addCommands(new RightTrenchCommand(Robot.m_rightTrenchGather, Robot.m_rightTrenchReturnBump, Robot.m_rightTrenchBumpGather));
        break;

      case AllianceConfigurationSubsystem.AUTO_MODE_RIGHT_BUMP:
        addCommands(new RightBumpCommand(Robot.m_rightBumpGather, Robot.m_rightBumpReturn, Robot.m_rightBumpGather2));
        break;

      case AllianceConfigurationSubsystem.AUTO_MODE_LEFT_BUMP:
        addCommands(new LeftBumpCommand(Robot.m_leftBumpGather, Robot.m_leftBumpReturn, Robot.m_leftBumpGather2
        ));
        break;

      case AllianceConfigurationSubsystem.AUTO_MODE_LEFT_TRENCH_RETURN_TRENCH:
        addCommands(new LeftTrenchCommand(Robot.m_leftTrenchGather, Robot.m_leftTrenchReturn, Robot.m_leftTrenchGather2));
        break;
      case AllianceConfigurationSubsystem.AUTO_MODE_LEFT_TRENCH_RETURN_BUMP:
        addCommands(new LeftTrenchCommand(Robot.m_leftTrenchGather, Robot.m_leftTrenchReturnBump, Robot.m_leftTrenchBumpGather));
        break;
      
        

      case AllianceConfigurationSubsystem.AUTO_MODE_DO_NOTHING:
      default:
        addCommands(
          new SequentialCommandGroup(
          new InstantCommand(
            () -> AllianceConfigurationSubsystem.setStartingPose(
              new Pose2d(
                AllianceConfigurationSubsystem.getStartLine(), 
                FieldConstants.yCenter, 
                new Rotation2d(AllianceConfigurationSubsystem.allianceAdjustedHeading(0.0)) // facing toward field center.
              )
            )
          ),
          new InstantCommand(() -> RobotContainer.m_robotDrive.setDrivingMode(DrivingMode.SPEED))
          //new InstantCommand(() -> RobotContainer.m_ShooterSubsystem.setRPM(3000.0)) // Initial guess at required rpm.  Could do better - TODO
        )
      );
    }
  }
}
