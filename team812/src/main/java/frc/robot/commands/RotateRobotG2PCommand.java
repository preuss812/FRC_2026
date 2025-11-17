// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.AllianceConfigurationSubsystem;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.utils.DrivingConfig;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateRobotG2PCommand extends GotoPoseCommand {

  private final double m_theta;
  private final boolean m_relative;
  private boolean debug = true;
  private Timer m_timer = new Timer();

  /*
   * RotateRobotG2PCommand - Creates a new RotateRobotG2PCommand.
   * @param robotDrive -  The drive subsystem to use.
   * @param poseEstimatorSubsystem - The pose estimator subsystem to use.
   * @param theta - The angle to rotate to in radians.
   * @param relative - If true, rotate relative to the current angle.  If false, rotate to the absolute angle.
   * @param config - The driving configuration to use.
   */
  public RotateRobotG2PCommand(
      DriveSubsystemSRX robotDrive
    , PoseEstimatorSubsystem poseEstimatorSubsystem
    , double theta
    , boolean relative
    , DrivingConfig config
    ) {
    super(robotDrive, poseEstimatorSubsystem, new Pose2d(), false, config); // Pose is a place holder.
    m_theta = theta;
    m_relative = relative;
    // Use addRequirements() here to declare subsystem dependencies.
    // Super call adds the drive subsystem as a requirement.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d startingPose = getPoseEstimatorSubsystem().getCurrentPose();
    double targetTheta;
    // The timer is used for tuning the PID parameters.
    if (debug) {
      m_timer.reset();
      m_timer.start();
    }
    
    // Calculate the target angle using absolute or relative depending on <relative>.
    // Ensure that the target is between -pi and pi to avoid rotating the long
    // way around.
    if (m_relative) {
      // get the robot's current rotation from the drivetrain
      double startingTheta = getRobotDrive().getPose().getRotation().getRadians(); 
      targetTheta = MathUtil.angleModulus(startingTheta + m_theta);
    } else {
      // set the target angle to the angle specified in the command.
      targetTheta = AllianceConfigurationSubsystem.allianceAdjustedTelopRotation(m_theta);
    }
    // Set the target pose changing only the orientation/rottation (ie not the X,Y position).
    setTargetPose(new Pose2d(startingPose.getTranslation(), new Rotation2d(targetTheta)));

    // Initialize the GotoPoseCommand with the new target pose.
    super.initialize();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (debug) SmartDashboard.putNumber("RR time", m_timer.get()); // Display how long the rotation took.
    if (debug) m_timer.stop();
    super.end(interrupted);
  }
  
}
