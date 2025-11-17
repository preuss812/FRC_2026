// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AllianceConfigurationSubsystem;
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
    Pose2d startingPose = RobotContainer.m_poseEstimatorSubsystem.getCurrentPose();

    Pose2d  allianceAdjustedRelativeMove = AllianceConfigurationSubsystem.allianceAdjustedMove(relativeMove);
    // add the relativeMove to the startingPose accounting for alliance color.
    setTargetPose(new Pose2d(
      startingPose.getX() + allianceAdjustedRelativeMove.getX(),
      startingPose.getY() + allianceAdjustedRelativeMove.getY(),
      startingPose.getRotation().rotateBy(allianceAdjustedRelativeMove.getRotation())
    ));

    // Now that we know the target pose, we can initialze the GotoPoseCommand.
    super.initialize();
  }

}
