// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveChoreoPathCommand;
import frc.robot.commands.FaceHubCommand;
import frc.robot.commands.FireAtWillCommand;
import frc.robot.commands.GotoPoseCommand;
import frc.robot.commands.ShooterTest;
import frc.robot.subsystems.AllianceConfigurationSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CenterShootCommand extends SequentialCommandGroup {
  /** Creates a new CenterShootCommand. */
  public CenterShootCommand() {
    addCommands(
      new ParallelCommandGroup(
        new InstantCommand(() -> RobotContainer.m_ShooterSubsystem.setRPM(3000.0)), // Start the flywheel spinning at an initial guess at the required rpm.  Could do better - TODO
        new DriveChoreoPathCommand(
          RobotContainer.m_robotDrive,
          RobotContainer.m_poseEstimatorSubsystem,
          "CenterShoot",
          RobotContainer.m_robotDrive.defaultAutoConfig,
          1.0,
          1.0)
      ),
      new ParallelRaceGroup(
        new FaceHubCommand(
          RobotContainer.m_robotDrive,
          RobotContainer.m_poseEstimatorSubsystem),
        new ShooterTest(RobotContainer.m_ShooterSubsystem, RobotContainer.m_poseEstimatorSubsystem),
        new FireAtWillCommand().withTimeout(10.0) // TODO fix FireAtWill to use ShooterSubsystem.canshoot() and check rotation of robot toward the hub.
        ),

        new GotoPoseCommand(
          RobotContainer.m_robotDrive,
          RobotContainer.m_poseEstimatorSubsystem,
          DriveConstants.robotLeftAtPose(
            RobotContainer.m_poseEstimatorSubsystem.getAprilTagPose(AllianceConfigurationSubsystem.getOutpostAprilTag().id()),
            0.0),
          RobotContainer.m_robotDrive.defaultAutoConfig
      )
    );
  }

}
