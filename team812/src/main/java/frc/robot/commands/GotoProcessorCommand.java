// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Utilities;
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

  /** Creates a new GotoProcessorCommand. */
  public GotoProcessorCommand(
      DriveSubsystemSRX DriveSubsystemSRXSubsystem
    , PoseEstimatorSubsystem PoseEstimatorSubsystem
    , DrivingConfig config
    ) { 
    super(DriveSubsystemSRXSubsystem, PoseEstimatorSubsystem, new Pose2d(), false, config); // Pose is a place holder.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    super.initialize();
    double offsetFromAprilTag = 0.0; // This should result in the robot front touching the wall where the april tag is.

    if (Utilities.isBlueAlliance()) {
      Pose2d tag = poseEstimatorSubsystem.getAprilTagPose(VisionConstants.AprilTag.BLUE_PROCESSOR.id());
      // This should position the robot back to the AMP touching the wall.
        targetPose = DriveConstants.robotFrontAtPose(tag, offsetFromAprilTag);
        //targetPose = new Pose2d(tag.getX(), tag.getY() - DriveConstants.kBackToCenterDistance, tag.getRotation().plus(new Rotation2d(Math.PI)));

    } else if (Utilities.isRedAlliance()) {
      Pose2d tag = poseEstimatorSubsystem.getAprilTagPose(VisionConstants.AprilTag.RED_PROCESSOR.id());
      // This should position the robot back to the AMP touching the wall.
      targetPose = DriveConstants.robotFrontAtPose(tag, offsetFromAprilTag);
      //targetPose = new Pose2d(tag.getX(), tag.getY() - DriveConstants.kBackToCenterDistance, tag.getRotation().plus(new Rotation2d(Math.PI)));
    }
    else {
      targetPose = poseEstimatorSubsystem.getCurrentPose(); // Hack:: if we dont know the alliance. Dont move. 
    }

  }
}
