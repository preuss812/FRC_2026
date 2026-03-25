    // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoCommands;

import java.util.Optional;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveChoreoPathCommand;
import frc.robot.commands.FaceHubAndShootCommand;
import frc.robot.commands.LowerIntakeCommand;
import frc.robot.commands.SetShooterSpeedCommand;
import frc.robot.commands.ShakeTheIntakeCommand;

public class LeftBumpCommand extends SequentialCommandGroup {
    private final double speedFactor = 0.5; // 1.0 would be full speed.
    /** Creates a new LeftTrenchCommand. */
    public LeftBumpCommand(
        Optional<Trajectory<SwerveSample>> leftBumpGather,
        Optional<Trajectory<SwerveSample>> leftBumpReturn,
        Optional<Trajectory<SwerveSample>> leftBumpGather2
    ) {
        addCommands(
            new LowerIntakeCommand(RobotContainer.m_IntakeDeploymentSubsystem).withTimeout(2.0), // The timeout is just for simulation.
            new InstantCommand(()->RobotContainer.m_IntakeSubsystem.runMotor(IntakeConstants.pickupFuelSpeed),RobotContainer.m_IntakeSubsystem),
            new DriveChoreoPathCommand(
                RobotContainer.m_robotDrive,
                RobotContainer.m_poseEstimatorSubsystem,
                leftBumpGather,
                RobotContainer.m_robotDrive.defaultAutoConfig,
                speedFactor,
                1.0
            ),
            // Stop the intake.
            //new InstantCommand(()->RobotContainer.m_IntakeSubsystem.stop(), RobotContainer.m_IntakeSubsystem),
            new SetShooterSpeedCommand(RobotContainer.m_ShooterSubsystem, RobotContainer.m_FeederSubsystem, 3000), 
            // Return to out alliance zone 
            new DriveChoreoPathCommand(
            RobotContainer.m_robotDrive,
            RobotContainer.m_poseEstimatorSubsystem,
            leftBumpReturn,
            RobotContainer.m_robotDrive.defaultAutoConfig,
            speedFactor,
            1.0),
        
            // Shoot the fuel cells we just picked up.
            new ParallelCommandGroup(
                new FaceHubAndShootCommand(),
                new ShakeTheIntakeCommand(RobotContainer.m_IntakeDeploymentSubsystem)
            ).withTimeout(6.0),

            // Make a pass back to the center for more fuel.
            new InstantCommand(()->RobotContainer.m_IntakeSubsystem.runMotor(IntakeConstants.pickupFuelSpeed),RobotContainer.m_IntakeSubsystem),
            new DriveChoreoPathCommand(
            RobotContainer.m_robotDrive,
            RobotContainer.m_poseEstimatorSubsystem,
            leftBumpGather2,
            RobotContainer.m_robotDrive.defaultAutoConfig,
            speedFactor,
            1.0),

            // Shoot the fuel cells we just picked up.
            new InstantCommand(()->RobotContainer.m_IntakeSubsystem.stop(), RobotContainer.m_IntakeSubsystem),
            new ParallelCommandGroup(
                new FaceHubAndShootCommand(),
                new ShakeTheIntakeCommand(RobotContainer.m_IntakeDeploymentSubsystem)
            ).withTimeout(10.0)
        );
    }
}
