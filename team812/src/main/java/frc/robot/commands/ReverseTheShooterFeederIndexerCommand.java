// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// Run the shooter & feeder & indexer backwards to attempt to clear fuel that might be stuck.
public class ReverseTheShooterFeederIndexerCommand extends Command {
  private final ShooterSubsystem m_ShooterSubsystem;
  private final FeederSubsystem m_FeederSubsystem;
  private final IndexerSubsystem m_IndexerSubsystem;
  /** Creates a new ReverseTheShooterFeederIndexerCommand. */
  public ReverseTheShooterFeederIndexerCommand(
    ShooterSubsystem shooterSubsystem,
    FeederSubsystem feederSubsystem,
    IndexerSubsystem indexerSubsystem
  ) {
    this.m_ShooterSubsystem = shooterSubsystem;
    this.m_FeederSubsystem = feederSubsystem;
    this.m_IndexerSubsystem = indexerSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, /*feederSubsystem, */indexerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_ShooterSubsystem.setRPM(-3000);
    //m_FeederSubsystem.setRPM(-6000);
    m_ShooterSubsystem.setShooterMode(ShooterConstants.ShooterMode.UNJAMMING);
    m_IndexerSubsystem.runMotor(-Constants.IndexerConstants.indexerPercentOutput);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_ShooterSubsystem.stop();
    //m_FeederSubsystem.stop();
    m_ShooterSubsystem.setShooterMode(ShooterConstants.ShooterMode.AUTO_RANGING);
    m_IndexerSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // Expecting whileTrue
  }
}
