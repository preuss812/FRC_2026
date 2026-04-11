// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.IntakeDeploymentConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeDeploymentSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShakeIntakeWithPauseCommand extends Command {
  private final IntakeDeploymentSubsystem m_intakeDeploymentSubsystem;
  private final double upPosition = Constants.IntakeDeploymentConstants.maxPosition/2;
  private final double downPosition = 0;
  private final int upCount = 50;
  private final int downCount = 5;
  private boolean goingUp = true;
  private int counter = 0;
  private enum mode  {GOING_UP, GOING_DOWN, FULLY_DOWN, HOLDING_UP};
  private mode currentMode;


  /** Creates a new HoldIntakeSteadyCommand. */
  public ShakeIntakeWithPauseCommand(IntakeDeploymentSubsystem intakeDeploymentSubsystem) {
    m_intakeDeploymentSubsystem = intakeDeploymentSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeDeploymentSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentMode = mode.GOING_UP;
    counter = 0;
    goingUp = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(currentMode) {
      case GOING_UP:
        m_intakeDeploymentSubsystem.setPosition(upPosition);
        currentMode = mode.HOLDING_UP;
        counter = 0;
        break;
      case HOLDING_UP:
        counter++;
        if(counter >= upCount) {
          counter = 0;
          currentMode = mode.GOING_DOWN;
        }
        break;
      case GOING_DOWN:
        counter = 0;
        m_intakeDeploymentSubsystem.setRPM(IntakeDeploymentConstants.kIntakeDeploymentDownRPM);
        currentMode = mode.FULLY_DOWN;
        break;
      case FULLY_DOWN:
        counter = 0;
        if (m_intakeDeploymentSubsystem.fullyLowered()) currentMode = mode.GOING_UP;
        break;
    }
    // counter++;
    // if (goingUp && counter >= upCount) {
    //     counter = 0;
    //     goingUp = false;
    //     m_intakeDeploymentSubsystem.setRPM(IntakeDeploymentConstants.kIntakeDeploymentDownRPM);;
    // } else if (!goingUp && m_intakeDeploymentSubsystem.fullyLowered()) {
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
