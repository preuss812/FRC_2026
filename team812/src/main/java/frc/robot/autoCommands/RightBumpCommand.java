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
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.AgitateIntakeCommand;
import frc.robot.commands.DriveChoreoPathCommand;
import frc.robot.commands.FaceHubAndShootCommand;
import frc.robot.commands.LowerIntakeCommand;
import frc.robot.commands.SetShooterSpeedCommand;
import frc.robot.commands.ShakeTheIntakeCommand;

public class RightBumpCommand extends SequentialCommandGroup {
    private final double speedFactor = 0.8; // 1.0 would be full speed.
    /** Creates a new RightBumpCommand. */
    public RightBumpCommand(
        Optional<Trajectory<SwerveSample>> rightBumpGather,
        Optional<Trajectory<SwerveSample>> rightBumpReturn,
        Optional<Trajectory<SwerveSample>> rightBumpGather2
    ) {
        addCommands(
            new LowerIntakeCommand(RobotContainer.m_IntakeDeploymentSubsystem).withTimeout(1.0), // The timeout is just for simulation.
            new InstantCommand(()->RobotContainer.m_IntakeSubsystem.runMotor(IntakeConstants.pickupFuelSpeed),RobotContainer.m_IntakeSubsystem),

            new DriveChoreoPathCommand(
                RobotContainer.m_robotDrive,
                RobotContainer.m_poseEstimatorSubsystem,
                rightBumpGather,
                RobotContainer.m_robotDrive.defaultAutoConfig,
                speedFactor,
                1.0
            ),
            // Stop the intake.
            new SetShooterSpeedCommand(RobotContainer.m_ShooterSubsystem, 3000), 
            // Return to out alliance zone 
            new DriveChoreoPathCommand(
            RobotContainer.m_robotDrive,
            RobotContainer.m_poseEstimatorSubsystem,
            rightBumpReturn,
            RobotContainer.m_robotDrive.defaultAutoConfig,
            speedFactor,
            1.0),
        
            // Shoot the fuel cells we just picked up.
            new ParallelCommandGroup(
                new FaceHubAndShootCommand(),
                new ShakeTheIntakeCommand(RobotContainer.m_IntakeDeploymentSubsystem),
                new AgitateIntakeCommand(RobotContainer.m_IntakeSubsystem))
                .withTimeout(Constants.AutoConstants.kShooterTimout),

            // Make a second pass through the center for more fuel.
            new InstantCommand(()->RobotContainer.m_IntakeSubsystem.runMotor(IntakeConstants.pickupFuelSpeed),RobotContainer.m_IntakeSubsystem),
            new DriveChoreoPathCommand(
            RobotContainer.m_robotDrive,
            RobotContainer.m_poseEstimatorSubsystem,
            rightBumpGather2,
            RobotContainer.m_robotDrive.defaultAutoConfig,
            speedFactor,
            1.0),

            // Shoot the fuel we just gathered.
            new InstantCommand(()->RobotContainer.m_IntakeSubsystem.stop(), RobotContainer.m_IntakeSubsystem),
            new ParallelCommandGroup(
                new FaceHubAndShootCommand(),
                new ShakeTheIntakeCommand(RobotContainer.m_IntakeDeploymentSubsystem),
                new AgitateIntakeCommand(RobotContainer.m_IntakeSubsystem))
            );
    }
}
