// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeDeploymentConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeDeploymentSubsystem;

public class LowerIntakeCommand extends Command {
  private final IntakeDeploymentSubsystem m_intakeDeploymentSubsystem;
  private boolean m_atBottom = false;
  private double m_startTime = 0.0;

  /** Creates a new LowerIntakeCommand. */
  public LowerIntakeCommand(IntakeDeploymentSubsystem intakeDeploymentSubsystem) {
    m_intakeDeploymentSubsystem = intakeDeploymentSubsystem;
    addRequirements(m_intakeDeploymentSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_atBottom = false;
    m_startTime = Timer.getFPGATimestamp();
    m_intakeDeploymentSubsystem.setHoldDown(true);
    // Use low-power duty cycle to lower, NEVER velocity PID into a hard stop
    m_intakeDeploymentSubsystem.runMotor(IntakeDeploymentConstants.kIntakeDeploymentLoweringDutyCycle);
    RobotContainer.setRaisingIntake(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double elapsedTime = Timer.getFPGATimestamp() - m_startTime;

    // Detect bottom either via reverse limit switch, motor stall, or after 0.8s max lowering
    if (!m_atBottom) {
      if (m_intakeDeploymentSubsystem.isRevLimitSwitchPressed()
          || (elapsedTime > 0.4 && Math.abs(m_intakeDeploymentSubsystem.getRPM()) < 50.0)
          || elapsedTime > 0.8) {
        m_atBottom = true;
      }
    }

    if (m_atBottom) {
      // Continuously apply gentle, current regulated holding torque to keep arm clamped down against hard stop
      m_intakeDeploymentSubsystem.holdDown();
    } else {
      m_intakeDeploymentSubsystem.runMotor(IntakeDeploymentConstants.kIntakeDeploymentLoweringDutyCycle);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Hold down mode is set, so stop() keeps the holding torque applied instead of cutting power.
    m_intakeDeploymentSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Keep running and holding torque down until interrupted (e.g. by RaiseIntakeCommand)
    return false;
  }
}
