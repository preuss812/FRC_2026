// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
/**
 * Accepts the RobotContainer as an argument and stops all the
 * motors on the robot and ends any commands scheduled to the
 * subsystems associated with the motors.
 */
public class StopAllMotorsCommand extends Command {

  /** Creates a new StopAllMotorsCommand. */
  public StopAllMotorsCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(
  
      RobotContainer.m_robotDrive
    );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_robotDrive.drive(0,0,0,true,false);
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
    return true;  // This is going to do all it's work in initialize so exit right away.
  }
}
