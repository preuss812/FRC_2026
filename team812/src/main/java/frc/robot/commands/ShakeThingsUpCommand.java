// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystemSRX;

// This command will possibly violently shake the robot to try and shake balls into the shooter. 
public class ShakeThingsUpCommand extends Command {
  private final DriveSubsystemSRX m_robotDrive;
  private double m_heading = 0;
  /** Creates a new ShakeThingsUpCommand. */
  public ShakeThingsUpCommand(DriveSubsystemSRX robotDrive) {
    m_robotDrive = robotDrive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_heading = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = Math.cos(m_heading);
    double y = Math.sin(m_heading);
    m_robotDrive.drive(x, y, 0.0, true, true);
    m_heading = MathUtil.angleModulus(m_heading + Math.PI/2.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_robotDrive.drive(0.0, 0.0, 0.0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // Expecting external termination eg .whileTrue or .withTimeout;
  }
}
