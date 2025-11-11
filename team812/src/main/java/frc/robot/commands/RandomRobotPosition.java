// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Random;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PoseEstimatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RandomRobotPosition extends Command {
  private Random rnd = new Random();
  private final PoseEstimatorSubsystem m_poseEstimatorSubsystem;
  /** Creates a new RandomRobotPosition. */
  public RandomRobotPosition(PoseEstimatorSubsystem poseEstimatorSubsystem) {
    m_poseEstimatorSubsystem = poseEstimatorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    // Intentionally not adding poseEstimatorSubsystem as a requirement to avoid conflicts.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_poseEstimatorSubsystem.setCurrentPose(
      new Pose2d(
        rnd.nextDouble() * (Constants.FieldConstants.fieldLength - 2.0) + 1 // Xposition
        ,rnd.nextDouble() * (Constants.FieldConstants.fieldWidth - 2.0) + 1 // Xposition
        ,new Rotation2d( rnd.nextDouble() * Math.PI) // Rotation     
      )
    );
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
