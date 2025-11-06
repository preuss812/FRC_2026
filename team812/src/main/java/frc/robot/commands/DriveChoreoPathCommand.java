// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.utils.DrivingConfig;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveChoreoPathCommand extends Command {

  private final String trajectoryName;
  private final DriveSubsystemSRX robotDrive;
  private final PoseEstimatorSubsystem poseEstimatorSubsystem;
  private Optional<Trajectory<SwerveSample>> trajectory;
  private final Timer timer = new Timer();

  /** Creates a new DriveChoreoPathCommand. */
  public DriveChoreoPathCommand(
    DriveSubsystemSRX robotDrive
  , PoseEstimatorSubsystem poseEstimatorSubsystem
  , String trajectoryName 
  , DrivingConfig config) {
    this.robotDrive = robotDrive;
    this.poseEstimatorSubsystem = poseEstimatorSubsystem;
    this.trajectoryName = trajectoryName;
    trajectory  = Choreo.loadTrajectory(trajectoryName);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(robotDrive, poseEstimatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
   if (trajectory.isPresent()) {
     //RobotContainer.m_PoseEstimatorSubsystem.field2d.getObject("trajectory").setTrajectory(trajectory.get());// wrong class of trajectory
            // Get the initial pose of the trajectory
            Optional<Pose2d> initialPose = trajectory.get().getInitialPose(isRedAlliance());

            if (initialPose.isPresent()) {
                // Reset odometry to the start of the trajectory
                robotDrive.resetOdometry(initialPose.get());
                if (trajectoryName == "PID test")
                {
                  // Set up at the wrong start location to see if the robot can correct itself
                  Pose2d  offsetPose = initialPose.get();
                  poseEstimatorSubsystem.setCurrentPose(new Pose2d(offsetPose.getX(), offsetPose.getY() + 2.0, offsetPose.getRotation()));
                }
                else              
                {  // Set the pose estimator to the start of the traject
                  poseEstimatorSubsystem.setCurrentPose(initialPose.get());
                
                }
            }
          }

        // Reset and start the timer when the autonomous period begins
        timer.restart();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

   if (trajectory.isPresent()) {
      // Sample the trajectory at the current time into the autonomous period
      Optional<SwerveSample> sample = trajectory.get().sampleAt(timer.get(), isRedAlliance());

      if (sample.isPresent()) {
          robotDrive.followTrajectory(sample.get());
      }
    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return trajectory.isPresent() && (timer.get() >= trajectory.get().getTotalTime());
  }

  private boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
    }
}
