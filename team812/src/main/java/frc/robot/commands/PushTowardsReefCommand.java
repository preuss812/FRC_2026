// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Autonomous;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.utils.PreussAutoDrive;

// This command just starts the robot spinning and keeps rotating at the requested speed.
// That means that some other outside force has to end this command.
public class PushTowardsReefCommand extends Command {

  private final PoseEstimatorSubsystem poseEstimatorSubsystem;
  private final PreussAutoDrive autoDrive;

  /** Creates a new RotateRobotCommand. */
  public PushTowardsReefCommand(DriveSubsystemSRX robotDrive, PoseEstimatorSubsystem poseEstimatorSubsystem) {
    this.poseEstimatorSubsystem = poseEstimatorSubsystem;
    autoDrive = new PreussAutoDrive(robotDrive, poseEstimatorSubsystem, robotDrive.defaultAutoConfig );
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    autoDrive.reset();
    // For debug, initialize the reef center if Autonomous was not started.
    Autonomous.setReefCenter();
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // I'm intentially not controlling rotation as the expected scenario is that we are already touching
    // the reef so the force toward the reef will rotate the robot to butt up against the reef. 
    double throttle = 0.11; // Minimal throttle
    Pose2d currentPose = poseEstimatorSubsystem.getCurrentPose();
    double heading = Autonomous.robotHeadingForCameraToReefCenter(currentPose.getX(), currentPose.getY())+ VisionConstants.rearCameraHeading;
    double xDrive = Math.cos(heading) * throttle;
    double yDrive = Math.sin(heading) * throttle;
    autoDrive.drive(xDrive, yDrive, 0, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    autoDrive.drive(0,0,0, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // Has to be terminated externally.
  }
}
