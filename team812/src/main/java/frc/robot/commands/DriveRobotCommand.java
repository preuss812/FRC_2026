// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotContainer;
import frc.robot.Utilities;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.utils.DrivingConfig;

/**
 * This comand drives the robot the specified distance and rotation.
 */
public class DriveRobotCommand extends GotoPoseCommand {

  private final Pose2d relativeMove;
  
  /** Creates a new DriveDistanceCommand. */
  public DriveRobotCommand(
    DriveSubsystemSRX robotDrive
  , PoseEstimatorSubsystem poseEstimatorSubsystem
  , Pose2d relativeMove
  , boolean driveFacingFinalPose
  , DrivingConfig config) {
    super(robotDrive, poseEstimatorSubsystem, relativeMove, driveFacingFinalPose, config); // The relative movePose will be overwritten in the initialize method.
    this.relativeMove = relativeMove;
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(robotDrive); // I think the super constructor does this.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Calculate the target pose based on the current location and relative move.
    Pose2d startingPose = RobotContainer.m_PoseEstimatorSubsystem.getCurrentPose();
    
    // add the relativeMove to the startingPose // There is an alliance component to this.   
    // I'm assuming the caller has handled it in the relativeMove.
    if (Utilities.isBlueAlliance()) {
      targetPose = new Pose2d(
        startingPose.getX() + relativeMove.getX(),
        startingPose.getY() + relativeMove.getY(),
        startingPose.getRotation().rotateBy(relativeMove.getRotation())
      );

    } else if (Utilities.isRedAlliance()) {
      // This just inverts the X and Y moves as the field this year is rotated about the center of the field.
      targetPose = new Pose2d(
        startingPose.getX() - relativeMove.getX(),
        startingPose.getY() - relativeMove.getY(),
        startingPose.getRotation().rotateBy(relativeMove.getRotation())
      );

    } else {
      targetPose = startingPose; // Do nothing if we dont have an alliance.
    }
    
    // Now that we know the target pose, we can initialze the GotoPoseCommand.
    super.initialize();
  }

}
