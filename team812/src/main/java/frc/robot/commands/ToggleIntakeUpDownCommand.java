// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeDeploymentConstants;
import frc.robot.subsystems.IntakeDeploymentSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ToggleIntakeUpDownCommand extends Command {
  private final IntakeDeploymentSubsystem m_IntakeDeploymentSubsystem;
  private static boolean raisingIntake = true;
  /** Creates a new ToggleIntakeUpDownCommand. */
  public ToggleIntakeUpDownCommand(IntakeDeploymentSubsystem intakeDeploymentSubsystem) {
    m_IntakeDeploymentSubsystem = intakeDeploymentSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeDeploymentSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("RaisingIntake", raisingIntake);
    if (raisingIntake) {
        m_IntakeDeploymentSubsystem.setRPM(IntakeDeploymentConstants.kIntakeDeploymentDownRPM);
        raisingIntake = false;
    } else {
      m_IntakeDeploymentSubsystem.setRPM(IntakeDeploymentConstants.kIntakeDeploymentUpRPM);
      raisingIntake = true;
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
