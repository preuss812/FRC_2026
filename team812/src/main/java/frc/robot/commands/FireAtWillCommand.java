// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FireAtWillCommand extends Command {
  /** Creates a new FireAtWillCommand. */
  private final ShooterSubsystem shooterSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final PoseEstimatorSubsystem poseEstimatorSubsystem;
  public FireAtWillCommand(
    ShooterSubsystem shooterSubsystem,
    IndexerSubsystem indexerSubsystem,
    PoseEstimatorSubsystem poseEstimatorSubsystem) {
      this.shooterSubsystem = shooterSubsystem;
      this.indexerSubsystem = indexerSubsystem;
      this.poseEstimatorSubsystem = poseEstimatorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (poseEstimatorSubsystem.facingHub(ShooterConstants.rotationTolerance)
      && shooterSubsystem.readyToShoot()) {
        indexerSubsystem.runMotor(IndexerConstants.indexerPercentOutput);
    } else {
      indexerSubsystem.stop();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexerSubsystem.stop();
  }

  /*
   * isFinished - this never finishes and must be managed externally
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
