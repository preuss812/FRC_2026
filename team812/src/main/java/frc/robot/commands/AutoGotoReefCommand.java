// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Autonomous;
import frc.robot.AutonomousPlans;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoGotoReefCommand extends GotoPoseCommand {
  private final PoseEstimatorSubsystem m_poseEstimatorSubsystem;

  /** Creates a new AutoGotoReefCommand. */
  public AutoGotoReefCommand(
     DriveSubsystemSRX robotDrive
    , PoseEstimatorSubsystem poseEstimatorSubsystem
  ){
    super(robotDrive, poseEstimatorSubsystem, new Pose2d(), robotDrive.defaultAutoConfig);
    m_poseEstimatorSubsystem = poseEstimatorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    // Super adds the requirements
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try{
      setTargetPose(AutonomousPlans.finalPoses.get(Autonomous.getAutoMode()));
    }
    catch(Exception e){
      setTargetPose(m_poseEstimatorSubsystem.getCurrentPose());
    }
    super.initialize();
  }

}
