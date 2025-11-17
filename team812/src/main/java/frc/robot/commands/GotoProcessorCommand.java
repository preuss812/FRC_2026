// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.AllianceConfigurationSubsystem;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.utils.DrivingConfig;

/**
 * This command drives the robot to the Processor for the robot's alliance.
 * It will eventually follow a pre-planned trajectory from anywhere on the
 * field to the Processor.  It should probably lower the arm and quiese the intake
 * shooter motors as well.
 * The anywhere on the field part is not implemented here yet as it has some issues.
 */
public class GotoProcessorCommand extends GotoPoseCommand {

  /*
   * GotoProcessorCommand Creates a new GotoProcessorCommand.
   * @param robotDrive - the drivetrain syubsystem.
   * @param poseEstimatorSubsystem - the pose estimator subsystem.
   * @param config - the driving speed configuration or null for the default config.
   */
  public GotoProcessorCommand(
      DriveSubsystemSRX robotDrive
    , PoseEstimatorSubsystem poseEstimatorSubsystem
    , DrivingConfig config
    ) { 
    super(robotDrive, poseEstimatorSubsystem, new Pose2d(), false, config); // Pose is a place holder.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    super.initialize();
    double offsetFromAprilTag = 0.0; // This should result in the robot front touching the wall where the april tag is.

    Pose2d tagPose = getPoseEstimatorSubsystem().getAprilTagPose(AllianceConfigurationSubsystem.getProcessorAprilTag().id());
    // This should position the robot front is touching the wall centered on the processor.
    setTargetPose(DriveConstants.robotFrontAtPose(tagPose, offsetFromAprilTag));
  }
}
