// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Autonomous;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PointCameraTowardReefCommand extends GotoPoseCommand {
  /** Creates a new PointCameraTowardReefCommand. */
  public PointCameraTowardReefCommand(DriveSubsystemSRX robotDrive, PoseEstimatorSubsystem poseEstimatorSubsystem) {
    
    super(robotDrive, poseEstimatorSubsystem, new Pose2d(), true, null );

    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Calculate the pose to face 
    Pose2d currentPose = poseEstimatorSubsystem.getCurrentPose();
    targetPose = new Pose2d(
      currentPose.getTranslation(),
      new Rotation2d(Autonomous.robotHeadingForCameraToReefCenter(currentPose.getX(), currentPose.getY()))
    );
    super.initialize();
  }

  
}
