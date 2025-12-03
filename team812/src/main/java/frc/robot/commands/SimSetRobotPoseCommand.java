// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SimSetRobotPoseCommand extends Command {

  private final DriveSubsystemSRX m_robotDrive;
  private final PoseEstimatorSubsystem m_poseEstimatorSubsystem;
  private final Pose2d m_pose;

  /** Creates a new SimSetRobotPoseCommand. */
  public SimSetRobotPoseCommand(DriveSubsystemSRX robotDrive, PoseEstimatorSubsystem poseEstimatorSubsystem, Pose2d pose) {
    m_robotDrive = robotDrive;
    m_poseEstimatorSubsystem = poseEstimatorSubsystem;
    m_pose = pose;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_robotDrive, m_poseEstimatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    simSetRobotPose(m_robotDrive, m_poseEstimatorSubsystem, m_pose);
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

  public static void simSetRobotPose(DriveSubsystemSRX robotDrive, PoseEstimatorSubsystem poseEstimatorSubsystem, Pose2d pose) {
    robotDrive.setAngleDegrees(pose.getRotation().getDegrees());
    robotDrive.resetOdometry(pose);
    poseEstimatorSubsystem.setCurrentPose(pose);
  }
}
