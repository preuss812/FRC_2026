// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;

/*
 * FaceHubAndShootCommand - A command that faces the hub and shoots.
 * This command will not stop so a timeout or .whileTrue() or similar must be used when scheduling it.
*/
public class FaceHubAndShootCommand extends ParallelCommandGroup {
  /** Creates a new FaceHubAndShootCommand. */
  public FaceHubAndShootCommand() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new FaceHubCommand(
        RobotContainer.m_robotDrive,
        RobotContainer.m_poseEstimatorSubsystem),
      new PrepareToShootCommand(RobotContainer.m_ShooterSubsystem, RobotContainer.m_FeederSubsystem, RobotContainer.m_poseEstimatorSubsystem),
      new FireAtWillCommand(
        RobotContainer.m_ShooterSubsystem,
        RobotContainer.m_FeederSubsystem,
        RobotContainer.m_IndexerSubsystem,
        RobotContainer.m_poseEstimatorSubsystem 
      )
    );
  }
}
