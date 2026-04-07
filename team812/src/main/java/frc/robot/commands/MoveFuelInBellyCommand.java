// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BellySubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveFuelInBellyCommand extends Command {
  private final BellySubsystem m_BellySubsystem;
  /** Creates a new MoveFuelInBellyCommand. */
  public MoveFuelInBellyCommand(BellySubsystem bellySubsystem) {
    m_BellySubsystem  = bellySubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_BellySubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_BellySubsystem.setRPM(100);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_BellySubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
