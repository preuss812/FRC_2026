// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TrajectoryPlans;
import frc.robot.subsystems.PoseEstimatorSubsystem;

/**
 * This command sets the robot pose in the PoeEstimatorSubsystem.
 * It is intended to support debugging by allowing placement of the robot in
 * various parts of the field from which other commands can be exercised.
 */
 public class SetCurrentPoseCommand extends Command {
  
  private final PoseEstimatorSubsystem poseEstimatorSubsystem;
  private static int i = 0;
  private static int j = 0;
  
  /** Creates a new SetCurrentPoseCommand. */
  public SetCurrentPoseCommand(PoseEstimatorSubsystem poseEstimatorSubsystem) {
    this.poseEstimatorSubsystem = poseEstimatorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(poseEstimatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // This is a hack to allow the robot to be placed in various parts of the field.
    poseEstimatorSubsystem.setCurrentPose(TrajectoryPlans.fieldSquareToPose(i,j));
    i++;
    if (i >= TrajectoryPlans.numXSquares) {
      // Move to the next row
      i = 0;
      j++;
      if (j >= TrajectoryPlans.numYSquares) {
        // Start back at the first square.
        i = 0;
        j = 0;
      }
    }
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
