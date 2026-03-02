// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AllianceConfigurationSubsystem;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.utils.PreussAutoDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

public class FaceHubCommand extends Command {
  /** Creates a new FaceHub. */
  private final DriveSubsystemSRX m_DriveSubsystemSRX;
  private final PoseEstimatorSubsystem m_PoseEstimatorSubsystem;
  private final PreussAutoDrive m_preussAutoDrive;
  public FaceHubCommand(DriveSubsystemSRX driveSubsystemSRX, PoseEstimatorSubsystem poseEstimatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DriveSubsystemSRX = driveSubsystemSRX;
    m_PoseEstimatorSubsystem = poseEstimatorSubsystem;
    m_preussAutoDrive = new PreussAutoDrive(driveSubsystemSRX, poseEstimatorSubsystem, driveSubsystemSRX.defaultAutoConfig);
    addRequirements(driveSubsystemSRX);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_preussAutoDrive.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentRotation = m_PoseEstimatorSubsystem.getCurrentPose().getRotation().getRadians();
    double desiredRotation = AllianceConfigurationSubsystem.robotFrontFacingHub().getRadians();
    double rotationError = MathUtil.angleModulus(currentRotation - desiredRotation);
    double rotationPercent = m_preussAutoDrive.calculateClampedRotation(rotationError);
    m_DriveSubsystemSRX.allianceRelativeDrive(0.0, 0.0, rotationPercent, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the robot when the command ends.
    m_DriveSubsystemSRX.drive(0.0,0.0, 0.0, true,false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
