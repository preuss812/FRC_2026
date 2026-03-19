// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ReverseShooterFeederCommand extends Command {
  private final ShooterSubsystem m_shooterSubsystem;
private final FeederSubsystem m_feederSubsystem;
  /** Creates a new ReverseShooterFeederCommand. */
  public ReverseShooterFeederCommand(
    ShooterSubsystem shooterSubsystem,
    FeederSubsystem feederSubsystem
  ) {
    m_shooterSubsystem = shooterSubsystem;
    m_feederSubsystem = feederSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, feederSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.setRPM(-3000);
    m_feederSubsystem.setRPM(-3000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stop();
    m_feederSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
