// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeDeploymentConstants;
import frc.robot.subsystems.IntakeDeploymentSubsystem;

/* 
 * ShakeTheIntakeCommand - run the intake up and down for small periods of time to shake the robot.
 * @param intakeDeploymentSubsystem - the subsystem for raising and lowering the intake.
 */
 public class ShakeTheIntakeCommand extends Command {
  private final IntakeDeploymentSubsystem m_IntakeDeploymentSubsystem;
  private int counter;
  private boolean goingUp = true;
  /** Creates a new ShakeTheIntakeCommand. */
  public ShakeTheIntakeCommand(IntakeDeploymentSubsystem intakeDeploymentSubsystem) {
    m_IntakeDeploymentSubsystem = intakeDeploymentSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeDeploymentSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    goingUp = true;
    counter = IntakeDeploymentConstants.shakeCycles;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    counter--;
    if (counter == 0) {
      goingUp = !goingUp;
      counter = IntakeDeploymentConstants.shakeCycles;
    }
    if (goingUp) {
      m_IntakeDeploymentSubsystem.setRPM(IntakeDeploymentConstants.kIntakeDeploymentUpRPM);
    } else {
      m_IntakeDeploymentSubsystem.setRPM(IntakeDeploymentConstants.kIntakeDeploymentUpRPM);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
