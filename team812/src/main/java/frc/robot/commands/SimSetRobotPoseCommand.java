// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;

/**
 * SimSetRobotPoseCommand
 * 
 * Command used during simulation to place the robot in the location and orientation
 * defined by the pose for ease of testing.
 */
public class SimSetRobotPoseCommand extends Command {

  private final Pose2d m_pose;

  /** Creates a new SimSetRobotPoseCommand. */
  public SimSetRobotPoseCommand(DriveSubsystemSRX robotDrive, PoseEstimatorSubsystem poseEstimatorSubsystem, Pose2d pose) {
    
    m_pose = pose;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(robotDrive, poseEstimatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.setRobotPose(m_pose);
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
    return true; // All happens in initialize;
  }

}
