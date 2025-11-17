// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.utils.DrivingConfig;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveChoreoPathCommand extends Command {

  private final String m_trajectoryName;
  private final DriveSubsystemSRX m_robotDrive;
  private final PoseEstimatorSubsystem m_poseEstimatorSubsystem;
  private Optional<Trajectory<SwerveSample>> m_trajectory;
  private final Timer m_timer = new Timer();
  private final double m_speedFactor; // Can be used to speed up or slow down the path following  1.0 = speed as defined in path.
  private int m_count = 0; // Used for simulating time in a way that allows for breakpoints during simulation.
  private PIDController[] pidControllers = new PIDController[3]; // X, Y, and Rotation

  /** Creates a new DriveChoreoPathCommand. */
  public DriveChoreoPathCommand(
    DriveSubsystemSRX robotDrive
  , PoseEstimatorSubsystem poseEstimatorSubsystem
  , String trajectoryName 
  , DrivingConfig config
  , double speedFactor
  , double pidCorrectionFactor) {
    this.m_robotDrive = robotDrive;
    this.m_poseEstimatorSubsystem = poseEstimatorSubsystem;
    this.m_trajectoryName = trajectoryName;
    this.m_speedFactor = speedFactor;
    m_trajectory  = Choreo.loadTrajectory(trajectoryName);
    pidControllers[0] = new PIDController(10.0 * pidCorrectionFactor, 0.0, 0.0);
    pidControllers[1] = new PIDController(10.0 * pidCorrectionFactor, 0.0, 0.0);
    pidControllers[2] = new PIDController(7.5  * pidCorrectionFactor, 0.0, 0.0);
    pidControllers[2].enableContinuousInput(-Math.PI, Math.PI); // For wrapping rotation.

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(robotDrive, poseEstimatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_count = 0;
    if (m_trajectory.isPresent()) {
     //RobotContainer.m_PoseEstimatorSubsystem.field2d.getObject("trajectory").setTrajectory(trajectory.get());// wrong class of trajectory
            // Get the initial pose of the trajectory
            Optional<Pose2d> initialPose = m_trajectory.get().getInitialPose(isRedAlliance());

            if (initialPose.isPresent()) {
                // Reset odometry to the start of the trajectory
                //robotDrive.resetOdometry(initialPose.get());

                m_poseEstimatorSubsystem.setCurrentPose(initialPose.get());
                //robotDrive.setAngleDegrees(initialPose.get().getRotation().getDegrees()+(Utilities.isRedAlliance() ? 180 : 0.0));

                if (m_trajectoryName == "PID test")
                {
                  // Set up at the wrong start location to see if the robot can correct itself
                  Pose2d  offsetPose = initialPose.get();
                  //poseEstimatorSubsystem.setCurrentPose(initialPose.get());
                  m_poseEstimatorSubsystem.setCurrentPose(new Pose2d(offsetPose.getX(), offsetPose.getY() + 2.0, new Rotation2d(Math.PI/2.0))); //offsetPose.getRotation()));
                }
                else              
                {  // Set the pose estimator to the start of the traject
                  m_poseEstimatorSubsystem.setCurrentPose(initialPose.get());
                
                }
            }
          }

        // Reset and start the timer when the autonomous period begins
        m_timer.restart();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

   if (m_trajectory.isPresent()) {
      // Sample the trajectory at the current time into the autonomous period
      Optional<SwerveSample> sample = m_trajectory.get().sampleAt(getTime(), isRedAlliance());

      if (sample.isPresent()) {
          m_robotDrive.followTrajectory(sample.get(),pidControllers, m_speedFactor);
      }
    }
    m_count++;
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_trajectory.isPresent() && (getTime() >= m_trajectory.get().getTotalTime());
  }

  private boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
  }

  /*
   * getTime - helper function to return time possible modified by the speed factor.
   * m_speedFactor is used to speed up or slow down the path following  1.0 = speed as defined in path.
   * isSimulation check is used to simulate time in a way that allows for breakpoints during simulation.
   * @return time to be used for trajectory sampling.
   */
  private double getTime() {
    double curTime = m_timer.get() * m_speedFactor;
    if (RobotContainer.isSimulation()) {
      curTime = m_count * 0.02 * m_speedFactor; // simulate a 20ms periodic update
    }
    return curTime;
  }

}
