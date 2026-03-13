// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveChoreoPathCommand;
import frc.robot.commands.FaceHubAndShootCommand;
import frc.robot.commands.GotoPoseCommand;
import frc.robot.commands.PrepareToShootCommand;
import frc.robot.subsystems.AllianceConfigurationSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CenterShootCommand extends SequentialCommandGroup {
  /** Creates a new CenterShootCommand. */
  public CenterShootCommand() {
    addCommands(
      new ParallelCommandGroup(
        new InstantCommand(() -> RobotContainer.m_ShooterSubsystem.setRPM(2500.0), RobotContainer.m_ShooterSubsystem), // Start the flywheel spinning at an initial guess at the required rpm.  Could do better - TODO
        new InstantCommand(() -> RobotContainer.m_FeederSubsystem.setRPM(2500.0), RobotContainer.m_FeederSubsystem), // Start the flywheel spinning at an initial guess at the required rpm.  Could do better - TODO
        new DriveChoreoPathCommand(
          RobotContainer.m_robotDrive,
          RobotContainer.m_poseEstimatorSubsystem,
          "CenterLeftShoot",
          RobotContainer.m_robotDrive.defaultAutoConfig,
          1.0,
          1.0)
      ),
      new FaceHubAndShootCommand().withTimeout(5.0), // TODO: tune the timeout to shoot 8 fuel cells.

      // This positions the robot in front of the outpost tag, so we can get fed by the human operator.
      new GotoPoseCommand(
        RobotContainer.m_robotDrive,
        RobotContainer.m_poseEstimatorSubsystem,
        DriveConstants.robotLeftAtPose(
          RobotContainer.m_poseEstimatorSubsystem.getAprilTagPose(AllianceConfigurationSubsystem.getOutpostAprilTag().id()),0.0),
        RobotContainer.m_robotDrive.aggressiveAutoConfig
      )
    );
  }
}
