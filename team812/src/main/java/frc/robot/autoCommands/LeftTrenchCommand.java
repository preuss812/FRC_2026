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
import frc.robot.subsystems.IntakeDeploymentSubsystem;
import frc.robot.subsystems.DriveSubsystemSRX.DrivingMode;

public class LeftTrenchCommand extends SequentialCommandGroup {
    private final double speedFactor = 0.8; // 1.0 would be full speed.
    /** Creates a new LeftTrenchCommand. */
    public LeftTrenchCommand(
         Optional<Trajectory<SwerveSample>> leftTrenchGather,
          Optional<Trajectory<SwerveSample>> leftTrenchReturn,
          Optional<Trajectory<SwerveSample>> leftTrenchGather2
    ) {
        addCommands(
            new ParallelCommandGroup(
                new LowerIntakeCommand(RobotContainer.m_IntakeDeploymentSubsystem).withTimeout(1.0), // The timeout is just for simulation.
                new InstantCommand(()->RobotContainer.m_IntakeSubsystem.runMotor(IntakeConstants.pickupFuelSpeed),RobotContainer.m_IntakeSubsystem),
                new InstantCommand(() -> RobotContainer.m_robotDrive.setDrivingMode(DrivingMode.SPEED))
            ),

            new DriveChoreoPathCommand(
                RobotContainer.m_robotDrive,
                RobotContainer.m_poseEstimatorSubsystem,
                leftTrenchGather,
                RobotContainer.m_robotDrive.defaultAutoConfig,
                speedFactor,
                1.0
            ),
            // Note: we are leaving the intake running on the way back.
            // Start shooter spinning so it is up to speed when we get back to our home zone.
            new SequentialCommandGroup (
                new SetShooterSpeedCommand(RobotContainer.m_ShooterSubsystem, RobotContainer.m_FeederSubsystem, 3000), 

                new DriveChoreoPathCommand(
                    RobotContainer.m_robotDrive,
                    RobotContainer.m_poseEstimatorSubsystem,
                    leftTrenchReturn,
                    RobotContainer.m_robotDrive.defaultAutoConfig,
                    speedFactor,
                    1.0
                )
            ),
        
            // Shoot the fuel cells we just picked 
            new ParallelCommandGroup(
                new FaceHubAndShootCommand(),
                new ShakeTheIntakeCommand(RobotContainer.m_IntakeDeploymentSubsystem))
                .withTimeout(8)
            ,
            
            //Gather balls a second time
            new DriveChoreoPathCommand(
                    RobotContainer.m_robotDrive,
                    RobotContainer.m_poseEstimatorSubsystem,
                    leftTrenchGather2,
                    RobotContainer.m_robotDrive.defaultAutoConfig,
                    speedFactor,
                    1.0
                )
        );
    }
}
