// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AllianceConfigurationSubsystem;
import frc.robot.subsystems.DriveSubsystemSRX;

// This command just starts the robot spinning and keeps rotating at the requested speed.
// That means that some other outside force has to end this command.
public class SpinRobotCommand extends Command {
  private final DriveSubsystemSRX m_robotDrive;
  private final double m_speed;
  private double m_allianceSpeed;
  /** Creates a new RotateRobotCommand. */
  public SpinRobotCommand(DriveSubsystemSRX robotDrive, double speed) {
    this.m_robotDrive = robotDrive;
    this.m_speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_allianceSpeed = AllianceConfigurationSubsystem.allianceAdjustedAutonomousRotation(m_speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_robotDrive.drive(0.0, 0.0, m_allianceSpeed, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_robotDrive.drive(0.0,0.0,0.0, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
