// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeDeploymentConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FireAtWillWithShakingCommand extends Command {
  /** Creates a new FireAtWillWithShakingCommand. */
  private final ShooterSubsystem shooterSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final PoseEstimatorSubsystem poseEstimatorSubsystem;
  private int shakeCycleCounter = 0;
  private int shakeCounter;
  private final int shakeCount = 5; // Shake for 0.1 seconds (20ms * 5 cycles);
  private final int shakeCycle = 50; // Repeat every 1 second (20ms * 50 cycles);

  public FireAtWillWithShakingCommand(
    ShooterSubsystem shooterSubsystem,
    IndexerSubsystem indexerSubsystem,
    PoseEstimatorSubsystem poseEstimatorSubsystem) {
      this.shooterSubsystem = shooterSubsystem;
      this.indexerSubsystem = indexerSubsystem;
      this.poseEstimatorSubsystem = poseEstimatorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexerSubsystem, RobotContainer.m_IntakeSubsystem, RobotContainer.m_IntakeDeploymentSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shakeCycleCounter = shakeCycle;
    shakeCounter = shakeCount;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Manage shaking
    shakeCycleCounter--;
    shakeCounter--;
    if (shakeCycleCounter <= 0) {
        shakeCycleCounter = shakeCycle;
        shakeCounter = shakeCount;
        RobotContainer.m_IntakeDeploymentSubsystem.setRPM(IntakeDeploymentConstants.kIntakeDeploymentUpRPM);
        RobotContainer.m_IntakeSubsystem.runMotor(IntakeConstants.pickupFuelSpeed);
    }
    if (shakeCounter <= 0) {
        RobotContainer.m_IntakeDeploymentSubsystem.setRPM(IntakeDeploymentConstants.kIntakeDeploymentUpSlowlyRPM);
        RobotContainer.m_IntakeSubsystem.stop();
    }

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
    RobotContainer.m_IntakeSubsystem.stop();
    RobotContainer.m_IntakeDeploymentSubsystem.stop();
  }

  /*
   * isFinished - this never finishes and must be managed externally
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
