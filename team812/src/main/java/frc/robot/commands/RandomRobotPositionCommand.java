// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Random;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;

/**
 * RandomRobotPositionCommand
 * 
 * Update the gyro, drivetrain, and pose estimator to a random location somewhere on the field.
 * This is useful for exercising various other commands during simulation.
 */
public class RandomRobotPositionCommand extends Command {
  private Random rnd = new Random();
  /** Creates a new RandomRobotPosition. */
  public RandomRobotPositionCommand(DriveSubsystemSRX robotDrive, PoseEstimatorSubsystem poseEstimatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(robotDrive, poseEstimatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d randomPose = 
      new Pose2d(
        rnd.nextDouble() * (Constants.FieldConstants.fieldLength - 2.0) + 1 // Xposition
        ,rnd.nextDouble() * (Constants.FieldConstants.fieldWidth - 2.0) + 1 // Xposition
        ,new Rotation2d( rnd.nextDouble() * Math.PI) // Rotation     
      )
    ;
    RobotContainer.setRobotPose(randomPose);
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
    return true;  // Always true.  The work is done in initialize.
  }
}
