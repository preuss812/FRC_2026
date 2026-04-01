// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetShooterSpeedCommand extends Command {
  private final ShooterSubsystem m_ShooterSubsystem;
  private final double m_RPM;
  /** Creates a new SetShooterSpeedCommand. */
  public SetShooterSpeedCommand(
    ShooterSubsystem shooterSubsystem,
    double RPM
  ) {
    m_ShooterSubsystem = shooterSubsystem;
    m_RPM = RPM;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShooterSubsystem.setRPM(m_RPM);
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
    return true; // This just sets the motor speeds and exits.
  }
}
