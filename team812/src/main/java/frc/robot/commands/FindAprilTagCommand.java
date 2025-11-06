// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;
//import frc.robot.commands.SpinRobotCommand;
//import frc.robot.commands.WaitToSeeAprilTagCommand;

/**
 * This command waits briefly to see an april tag and then
 * rotates the robot to look for an april tag.
 * Once an april tag is in view, the command ends.
 * Typically, this would need a timeout in case no april tag is in view. 
 * The assumption is that the Pose Estimator is not capuring positions
 * so we are relying only on the drive subsystem to move the robot.
 */
public class FindAprilTagCommand extends SequentialCommandGroup {
  /** Creates a new FindAprilTagCommand. */
  public FindAprilTagCommand(DriveSubsystemSRX robotDrive, PoseEstimatorSubsystem poseEstimatorSubsystem, double speed) {
    // Start the robot spining and rotate until an april tag is in view.
        addCommands(
      new ParallelDeadlineGroup(
        new WaitToSeeAprilTagCommand(poseEstimatorSubsystem),  // This ends, it will stop the rotation.
        new SpinRobotCommand(robotDrive, speed)
      )
    );
  }
}
