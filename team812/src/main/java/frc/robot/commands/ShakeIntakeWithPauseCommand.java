// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeDeploymentSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShakeIntakeWithPauseCommand extends Command {
  private final IntakeDeploymentSubsystem m_intakeDeploymentSubsystem;
  private final double upPosition = Constants.IntakeDeploymentConstants.maxPosition/2;
  private final double downPosition = Constants.IntakeDeploymentConstants.maxPosition/4;
  private final int upCount = 50;
  private final int downCount = 5;
  private boolean goingUp = true;
  private int counter = 0;


  /** Creates a new HoldIntakeSteadyCommand. */
  public ShakeIntakeWithPauseCommand(IntakeDeploymentSubsystem intakeDeploymentSubsystem) {
    m_intakeDeploymentSubsystem = intakeDeploymentSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeDeploymentSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeDeploymentSubsystem.setPosition(upPosition);
    counter = 0;
    goingUp = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //  if (m_intakeDeploymentSubsystem.getPosition() <= Constants.IntakeDeploymentConstants.kIntakeDeploymentLoweredPosition) {
    //    m_intakeDeploymentSubsystem.setRPM(-10);
    //  }
    // counter++;
    // if (goingUp && counter >= upCount) {
    //     counter = 0;
    //     goingUp = false;
    //     m_intakeDeploymentSubsystem.setPosition(downPosition);
    // } else if (!goingUp && counter >= downCount) {
    //     counter = 0;
    //     goingUp = true;
    //     m_intakeDeploymentSubsystem.setPosition(upPosition);
    // }
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeDeploymentSubsystem.stop();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; //m_intakeDeploymentSubsystem.fullyLowered();
  }
}
