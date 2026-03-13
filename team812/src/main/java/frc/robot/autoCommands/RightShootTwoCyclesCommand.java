// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveChoreoPathCommand;
import frc.robot.commands.FaceHubAndShootCommand;
import frc.robot.commands.LowerIntakeCommand;

public class RightShootTwoCyclesCommand extends SequentialCommandGroup {
  /** Creates a new RightShootTwoCyclesCommand. */
  public RightShootTwoCyclesCommand() {
    addCommands(
        new ParallelCommandGroup(

            new LowerIntakeCommand(RobotContainer.m_IntakeDeploymentSubsystem).withTimeout(1.5),
            new InstantCommand(()->RobotContainer.m_IntakeSubsystem.runMotor(IntakeConstants.pickupFuelSpeed), RobotContainer.m_IntakeSubsystem),
            new DriveChoreoPathCommand(
                RobotContainer.m_robotDrive,
                RobotContainer.m_poseEstimatorSubsystem,
                "RightGather",
                RobotContainer.m_robotDrive.defaultAutoConfig,
                1.0,
                1.0
        )
      ),
      new ParallelCommandGroup(
        new InstantCommand(() -> RobotContainer.m_ShooterSubsystem.setRPM(3000.0), RobotContainer.m_ShooterSubsystem), // Start the flywheel spinning at an initial guess at the required rpm.  Could do better - TODO
        new InstantCommand(() -> RobotContainer.m_FeederSubsystem.setRPM(3000.0), RobotContainer.m_FeederSubsystem), // Start the flywheel spinning at an initial guess at the required rpm.  Could do better - TODO
        new InstantCommand(() -> RobotContainer.m_IntakeSubsystem.stop(), RobotContainer.m_IntakeSubsystem),
        new DriveChoreoPathCommand(
          RobotContainer.m_robotDrive,
          RobotContainer.m_poseEstimatorSubsystem,
          "RightReturnShoot",
          RobotContainer.m_robotDrive.defaultAutoConfig,
          1.0,
          1.0)
      ),
      new FaceHubAndShootCommand().withTimeout(6.0), // TODO: tune the timeout to shoot 8 fuel cells.
    new ParallelCommandGroup(
        new InstantCommand(() -> RobotContainer.m_ShooterSubsystem.stop(), RobotContainer.m_ShooterSubsystem),
        new InstantCommand(() -> RobotContainer.m_FeederSubsystem.stop(), RobotContainer.m_FeederSubsystem),
        new InstantCommand(()->RobotContainer.m_IntakeSubsystem.runMotor(IntakeConstants.pickupFuelSpeed), RobotContainer.m_IntakeSubsystem),
        new DriveChoreoPathCommand(
          RobotContainer.m_robotDrive,
          RobotContainer.m_poseEstimatorSubsystem,
          "RightGather2",
          RobotContainer.m_robotDrive.defaultAutoConfig,
          1.0,
          1.0)
      ),
      new ParallelCommandGroup(
        new InstantCommand(() -> RobotContainer.m_IntakeSubsystem.stop(), RobotContainer.m_IntakeSubsystem),
        new InstantCommand(() -> RobotContainer.m_ShooterSubsystem.setRPM(3000.0), RobotContainer.m_ShooterSubsystem), // Start the flywheel spinning at an initial guess at the required rpm.  Could do better - TODO
        new InstantCommand(() -> RobotContainer.m_FeederSubsystem.setRPM(3000.0), RobotContainer.m_FeederSubsystem), // Start the flywheel spinning at an initial guess at the required rpm.  Could do better - TODO
        new DriveChoreoPathCommand(
          RobotContainer.m_robotDrive,
          RobotContainer.m_poseEstimatorSubsystem,
          "RightReturnShoot2",
          RobotContainer.m_robotDrive.defaultAutoConfig,
          1.0,
          1.0)
      ),
      new FaceHubAndShootCommand().withTimeout(6.0) // TODO: tune the timeout 
    );
  }
}
