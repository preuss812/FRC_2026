// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoCommands;

import java.util.Optional;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveChoreoPathCommand;
import frc.robot.commands.FaceHubAndShootCommand;
import frc.robot.commands.LowerIntakeCommand;
import frc.robot.commands.SetShooterSpeedCommand;

public class RightTrenchCommand extends SequentialCommandGroup {
    private final double speedFactor = 0.5; // 1.0 would be full speed.
    /** Creates a new RightTrenchCommand. */
    public RightTrenchCommand(
        Optional<Trajectory<SwerveSample>> rightTrenchGather,
        Optional<Trajectory<SwerveSample>> rightTrenchReturn
    ) {
        addCommands(
            new LowerIntakeCommand(RobotContainer.m_IntakeDeploymentSubsystem).withTimeout(1.0), // The timeout is just for simulation.
            
            // Simultaneously drive out on the field and start the intake.
            new SequentialCommandGroup( 
                new InstantCommand(()->RobotContainer.m_IntakeSubsystem.runMotor(IntakeConstants.pickupFuelSpeed),RobotContainer.m_IntakeSubsystem),
                new DriveChoreoPathCommand(
                    RobotContainer.m_robotDrive,
                    RobotContainer.m_poseEstimatorSubsystem,
                    rightTrenchGather,
                    RobotContainer.m_robotDrive.defaultAutoConfig,
                    speedFactor,
                    1.0
                )
            ),
            // Stop the intake.
            //new InstantCommand(()->RobotContainer.m_IntakeSubsystem.stop(), RobotContainer.m_IntakeSubsystem),
            new SequentialCommandGroup(
                
                new SetShooterSpeedCommand(RobotContainer.m_ShooterSubsystem, RobotContainer.m_FeederSubsystem, 3000), 
                // Return to out alliance zone 
                new DriveChoreoPathCommand(
                RobotContainer.m_robotDrive,
                    RobotContainer.m_poseEstimatorSubsystem,
                    rightTrenchReturn,
                    RobotContainer.m_robotDrive.defaultAutoConfig,
                    speedFactor,
                    1.0
                )
            ),
        
            // Shoot the fuel cells we just picked up.
            new FaceHubAndShootCommand()
            
        );
    }
}
