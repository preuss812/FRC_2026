// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveChoreoPathCommand;
import frc.robot.commands.FaceHubAndShootCommand;
import frc.robot.commands.StandStillCommand;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FarRightShootOutpostShootCommand extends SequentialCommandGroup {
  /** Creates a new FarRightShootOutpostShootCommand. */
  public FarRightShootOutpostShootCommand() {
    addCommands(
      new ParallelCommandGroup(
        //new InstantCommand(() -> RobotContainer.m_ShooterSubsystem.setRPM(3000.0)), // Start the flywheel spinning at an initial guess at the required rpm.  Could do better - TODO
        new DriveChoreoPathCommand(
          RobotContainer.m_robotDrive,
          RobotContainer.m_poseEstimatorSubsystem,
          "FarRightToShoot",
          RobotContainer.m_robotDrive.defaultAutoConfig,
          1.0,
          1.0,
          true,
          true
        )
      ),
      new FaceHubAndShootCommand().withTimeout(6.0), // TODO: tune the timeout to shoot 8 fuel cells.

      // Drive to mid field and collect more fuel cells from the trench run and return to a viable shooting position.
      new DriveChoreoPathCommand(
        RobotContainer.m_robotDrive,
        RobotContainer.m_poseEstimatorSubsystem,
        "FarRtShootToOutpost",
        RobotContainer.m_robotDrive.defaultAutoConfig,
        1.0,
        1.0,
        false,
        false
      ),
      new StandStillCommand(RobotContainer.m_robotDrive).withTimeout(6.0),
      new DriveChoreoPathCommand(
      RobotContainer.m_robotDrive,
        RobotContainer.m_poseEstimatorSubsystem,
        "OutpostToShoot",
        RobotContainer.m_robotDrive.defaultAutoConfig,
        1.0,
        1.0,
        false,
        true // True starts the shooter with the speed for the first shot at the end of the trajectory.
      ),
      // Shoot the fuel cells we just picked up.
      new FaceHubAndShootCommand().withTimeout(6.0) // TODO: tune the timeout.

      
    );
  }
}
