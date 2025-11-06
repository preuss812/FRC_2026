// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BlackBoxSubsystem;
import frc.robot.subsystems.DriveSubsystemSRX;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BlackBoxDriveTestCommand extends Command {

  private final BlackBoxSubsystem blackBoxSubsystem;
  private final DriveSubsystemSRX robotDrive;

  /** Creates a new BlackBoxDriveTestCommand. */
  public BlackBoxDriveTestCommand(DriveSubsystemSRX robotDrive, BlackBoxSubsystem blackBoxSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.robotDrive = robotDrive;
    this.blackBoxSubsystem = blackBoxSubsystem;
    addRequirements(robotDrive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = blackBoxSubsystem.getPotValueScaled(0, -1.0, 1.0);
    double ySpeed = blackBoxSubsystem.getPotValueScaled(1, -1.0, 1.0);
    SmartDashboard.putNumber("BB x", xSpeed);
    SmartDashboard.putNumber("BB y", ySpeed);
    if (blackBoxSubsystem.isSwitchRight())
      robotDrive.drive(xSpeed, ySpeed, 0.0, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    robotDrive.drive(0, 0, 0.0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return blackBoxSubsystem.isSwitchLeft();
  }
}
