// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.Autonomous;
import frc.robot.AutonomousPlans;
import frc.robot.Constants.AutoConstants;
import frc.robot.TrajectoryPlans;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoSwerveToReefCommand extends PreussSwerveControllerCommand {
  private final PoseEstimatorSubsystem m_poseEstimatorSubsystem;
  
  /** Creates a new AutoDriveToReefCommand. */
  public AutoSwerveToReefCommand(
    DriveSubsystemSRX robotDrive,
    PoseEstimatorSubsystem poseEstimatorSubsystem
  ) {
    super(
      robotDrive,
      poseEstimatorSubsystem,
      new Trajectory(),
      () -> poseEstimatorSubsystem.getCurrentPose(), // Functional interface to feed supplier
      new PIDController(AutoConstants.kPXController, 0, 0),
      new PIDController(AutoConstants.kPYController, 0, 0),
      new ProfiledPIDController(
      AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints),
      () -> TrajectoryPlans.reefFacingRotationSupplier(poseEstimatorSubsystem),
      robotDrive::driveFieldRelative,
      robotDrive);
    m_poseEstimatorSubsystem = poseEstimatorSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    // Super will add : addRequirements(robotDrive, poseEstimatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Note: potential for a null pointer issue here if getAutoMode does not return a valid index.
    try {
      super.setTrajectory(AutonomousPlans.trajectories.get(Autonomous.getAutoMode()));
      // Reposition the robot logically to the ideal starting position based on the current autonomous mode.
      m_poseEstimatorSubsystem.setCurrentPose(AutonomousPlans.startingPoses.get(Autonomous.getAutoMode()));
    }
    catch(Exception e) {
      super.setTrajectory(new Trajectory()); 
    }
    super.initialize();
   
  }

}
