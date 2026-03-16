// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveChoreoPathCommand;
import frc.robot.commands.FaceHubAndShootCommand;
import frc.robot.commands.LowerIntakeCommand;
import frc.robot.commands.SetShooterSpeedCommand;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RightTrenchCommand extends SequentialCommandGroup {
    private final double speedFactor = 0.2; // 1.0 would be full speed.
    /** Creates a new RightTrenchCommand. */
    public RightTrenchCommand() {
        addCommands(
            new LowerIntakeCommand(RobotContainer.m_IntakeDeploymentSubsystem).withTimeout(2.0), // The timeout is just for simulation.
            new InstantCommand(()->RobotContainer.m_IntakeSubsystem.runMotor(IntakeConstants.pickupFuelSpeed),RobotContainer.m_IntakeSubsystem),
            new DriveChoreoPathCommand(
                RobotContainer.m_robotDrive,
                RobotContainer.m_poseEstimatorSubsystem,
                "RightTrenchGather",
                RobotContainer.m_robotDrive.defaultAutoConfig,
                speedFactor,
                1.0
            ),
            // Stop the intake.
            new InstantCommand(()->RobotContainer.m_IntakeSubsystem.stop(), RobotContainer.m_IntakeSubsystem),
            new SetShooterSpeedCommand(RobotContainer.m_ShooterSubsystem, RobotContainer.m_FeederSubsystem, 3000), 
            // Return to out alliance zone 
            new DriveChoreoPathCommand(
            RobotContainer.m_robotDrive,
            RobotContainer.m_poseEstimatorSubsystem,
            "RightTrenchReturn",
            RobotContainer.m_robotDrive.defaultAutoConfig,
            speedFactor,
            1.0),
        
            // Shoot the fuel cells we just picked up.
            new FaceHubAndShootCommand()
        );
    }
}
