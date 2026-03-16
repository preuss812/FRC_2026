// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeDeploymentSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShakeIntakeCommand extends Command {
  private final IntakeDeploymentSubsystem m_intakeDeploymentSubsystem;
  private boolean goingUp;
  private final int upCount = 5;
  private final int downCount = 4;
  private int m_count = 0;

  /** Creates a new ShakeIntakeCommand. */
  public ShakeIntakeCommand(IntakeDeploymentSubsystem intakeDeploymentSubsystem) {
    m_intakeDeploymentSubsystem = intakeDeploymentSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeDeploymentSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeDeploymentSubsystem.setRPM(Constants.IntakeDeploymentConstants.kIntakeDeploymentShakeUpRPM);
    m_count = upCount;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_count--;
    if (m_count == 0) {
        if (goingUp) {
            goingUp = false;
            m_count = downCount;
            m_intakeDeploymentSubsystem.setRPM(Constants.IntakeDeploymentConstants.kIntakeDeploymentShakeDownRPM);
        } else {
            goingUp = true;
            m_count = upCount;
            m_intakeDeploymentSubsystem.setRPM(Constants.IntakeDeploymentConstants.kIntakeDeploymentShakeUpRPM);

        }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeDeploymentSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // use whileTrue or withTimeout.
  }
}
