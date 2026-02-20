// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TrajectoryPlans;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.utils.PreussAutoDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

public class DriveFacingHub extends Command {
  /** Creates a new FaceHub. */
  private final DriveSubsystemSRX m_DriveSubsystemSRX;
  private final PoseEstimatorSubsystem m_PoseEstimatorSubsystem;
  private final XboxController m_XboxController;
  private final PreussAutoDrive m_preussAutoDrive;
  public DriveFacingHub(DriveSubsystemSRX driveSubsystemSRX, PoseEstimatorSubsystem poseEstimatorSubsystem, XboxController xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DriveSubsystemSRX = driveSubsystemSRX;
    m_PoseEstimatorSubsystem = poseEstimatorSubsystem;
    m_XboxController = xboxController;
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
    double x = -MathUtil.applyDeadband(m_XboxController.getLeftY(), OIConstants.kDriveDeadband);
    double y = -MathUtil.applyDeadband(m_XboxController.getLeftX(), OIConstants.kDriveDeadband);
    double currentRotation = m_PoseEstimatorSubsystem.getCurrentPose().getRotation().getRadians();
    double desiredRotation = TrajectoryPlans.robotFrontFacingHub().getRadians();
    double rotationError = MathUtil.angleModulus(currentRotation - desiredRotation);
    double rotationPercent = m_preussAutoDrive.calculateClampedRotation(rotationError);
    m_DriveSubsystemSRX.allianceRelativeDrive(x, y, rotationPercent, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_XboxController.getRightX()) > 0.1;
  }
}
