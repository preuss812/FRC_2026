// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Command to delay the start of autonomous motion for some number of seconds.
 * This might be needed to avoid collisions during autonomous mode if a 
 * teammate is driving in the same area.
 */
public class AutonomousStartDelayCommand extends Command {
  /** Creates a new AutonomousStartDelayCommand. */
  private double delay = 0.0;

  public AutonomousStartDelayCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    delay = SmartDashboard.getNumber("AutoStartDelay",0);
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
    double matchTime = DriverStation.getMatchTime();
    return matchTime >= delay;
  }
}