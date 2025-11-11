// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Autonomous;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.TrajectoryPlans;
import frc.robot.Utilities;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

/**
 * AutoDriveToReefCommand - return a command for autonomous driving from the start line to the reef
 * based on Field Management alliance (red or blue) and the autonomous plan selected.
 */
public class AutoDriveToReefCommand extends SequentialCommandGroup {
  
  /** Creates a new AutoDriveToReefCommand. */
  public AutoDriveToReefCommand(
    DriveSubsystemSRX robotDrive
    , PoseEstimatorSubsystem poseEstimatorSubsystem
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    // find the april tag id if things are initialized.
    int apriltagId =  VisionConstants.NO_TAG_FOUND;
    if (Autonomous.m_autoMode >= 0 && Autonomous.m_autoMode < TrajectoryPlans.expectedAprilTag.size()) {
      if (TrajectoryPlans.expectedAprilTag.get(Autonomous.m_autoMode) != null) {
        apriltagId = TrajectoryPlans.expectedAprilTag.get(Autonomous.m_autoMode);
        if (apriltagId < VisionConstants.MIN_FIDUCIAL_ID || apriltagId > VisionConstants.MAX_FIDUCIAL_ID) {
          apriltagId =  VisionConstants.NO_TAG_FOUND;
        } else {
          // If we are the red alliance, we need to get the red alliance's complementary tag id.
          if (Utilities.isRedAlliance()) {
            apriltagId = FieldConstants.complementaryAprilTag[apriltagId];
          }
        }
      }

    }
    addCommands(
      new InstantCommand(() ->SmartDashboard.putString("AutoCommand", "setStartingPose")),
      TrajectoryPlans.setStartingPoseCommand(poseEstimatorSubsystem)
      //new WaitCommand(2.0),
      //new InstantCommand(() ->SmartDashboard.putString("AutoCommand", "PointTowardReef"))
    );
    /*
    if (apriltagId == VisionConstants.NO_TAG_FOUND) {
      addCommands(new PointCameraTowardReefCommand(robotDrive, poseEstimatorSubsystem));
    } else {
      addCommands(new PointCameraTowardApriltagCommand(robotDrive, poseEstimatorSubsystem, apriltagId));
    }
    */
    addCommands(
      //new StandStillCommand(robotDrive).withTimeout(0.1),
      //new WaitCommand(2.0),
      new InstantCommand(() ->SmartDashboard.putString("AutoCommand", "SwerveToReef")),
      TrajectoryPlans.getReefFacingSwerveCommand(robotDrive, poseEstimatorSubsystem),
      //new StandStillCommand(robotDrive).withTimeout(0.1),
      //new WaitCommand(2.0),

      new InstantCommand(() ->SmartDashboard.putString("AutoCommand", "GotoReefPose")),
      TrajectoryPlans.gotoFinalPoseCommand(robotDrive, poseEstimatorSubsystem), //.withTimeout(5.0),
      //new StandStillCommand(robotDrive).withTimeout(0.1)
      new InstantCommand(() ->SmartDashboard.putString("AutoCommand", "driveDone"))

    );
  }
}
