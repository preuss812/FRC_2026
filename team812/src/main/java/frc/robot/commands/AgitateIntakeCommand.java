// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AgitateIntakeCommand extends Command {
  private final IntakeSubsystem m_IntakeSubsystem;
  private boolean runningForward = false;
  private Timer timer = new Timer();
  private double timeToSwitch = IntakeConstants.agitateReversedTime;
  /** Creates a new AgitateIntake. */
  public AgitateIntakeCommand(IntakeSubsystem intakeSubsystem) {
    m_IntakeSubsystem = intakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      runningForward = false;
      timeToSwitch = IntakeConstants.agitateReversedTime;
      timer.reset();
      timer.start();
      m_IntakeSubsystem.runMotor(-IntakeConstants.pickupFuelSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.hasElapsed(timeToSwitch)) {
      if(runningForward) {
        m_IntakeSubsystem.runMotor(-IntakeConstants.pickupFuelSpeed);
        timeToSwitch += IntakeConstants.agitateReversedTime;
      } else {
        m_IntakeSubsystem.runMotor(IntakeConstants.pickupFuelSpeed);
        timeToSwitch += IntakeConstants.agitateForwardTime;
      }
      runningForward = !runningForward;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.stop();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
