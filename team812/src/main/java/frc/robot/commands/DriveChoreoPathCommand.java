// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.ListIterator;
import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.utils.DrivingConfig;

public class DriveChoreoPathCommand extends Command {

  private final String m_trajectoryName;
  private final DriveSubsystemSRX m_robotDrive;
  private final PoseEstimatorSubsystem m_poseEstimatorSubsystem;
  private Optional<Trajectory<SwerveSample>> m_trajectory;
  private final Timer m_timer = new Timer();
  private final double m_speedFactor; // Can be used to speed up or slow down the path following  1.0 = speed as defined in path.
  private int m_count = 0; // Used for simulating time in a way that allows for breakpoints during simulation.
  private PIDController[] pidControllers = new PIDController[3]; // X, Y, and Rotation
  private boolean debug = true;
  private Optional<Pose2d> m_initialPose = Optional.empty();

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
    m_initialPose = m_trajectory.get().getInitialPose(isRedAlliance());

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

            if (m_initialPose.isPresent()) {
                // Reset odometry to the start of the trajectory
                //robotDrive.resetOdometry(initialPose.get());

                RobotContainer.setRobotPose(m_initialPose.get());

                if (m_trajectoryName == "PID test")
                {
                  // Set up at the wrong start location to see if the robot can correct itself
                  Pose2d  offsetPose = m_initialPose.get();
                  //poseEstimatorSubsystem.setCurrentPose(initialPose.get());
                  m_poseEstimatorSubsystem.setCurrentPose(new Pose2d(offsetPose.getX(), offsetPose.getY() + 2.0, new Rotation2d(Math.PI/2.0))); //offsetPose.getRotation()));
                }
                else              
                {  // Set the pose estimator to the start of the traject
                  m_poseEstimatorSubsystem.setCurrentPose(m_initialPose.get());
                
                }
            }
            RobotContainer.m_poseEstimatorSubsystem.field2d.getObject("trajectory").setTrajectory(choreoToWPITrajectory(m_trajectory));
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
        followTrajectory(sample.get(),pidControllers, m_speedFactor);
      }
    }
    m_count++;
  
  }

  /**
   * followTrajectory - Drive the robot to the next point in the trajectory.
   * @param sample - the current trajectory sample which has the desired speed and location for the current time.
   * @param pidControllers - pid controllers for x, y, and theta of the robot.
   * @param speedFactor - from 0.0 .. 1.0 the percentage of speed required from the trajectory file.
   */
  public void followTrajectory(SwerveSample sample, PIDController[] pidControllers, double speedFactor) {
    // Get the current pose of the robot
    Pose2d pose = m_poseEstimatorSubsystem.getCurrentPose();
    double xFF = sample.vx * speedFactor;
    double yFF = sample.vy * speedFactor;
    double omegaFF =  sample.omega * speedFactor;
    double xFeedback = pidControllers[0].calculate(pose.getX(), sample.x);
    double yFeedback = pidControllers[1].calculate(pose.getY(), sample.y);
    double omegaFeedback = pidControllers[2].calculate(pose.getRotation().getRadians(), sample.heading);

    // Generate the next speeds for the robot
    ChassisSpeeds speeds = new ChassisSpeeds(
      xFF + xFeedback,
      yFF + yFeedback,
      omegaFF + omegaFeedback
    );

    if (debug) {
      SmartDashboard.putNumber("xFeedback", xFeedback);
      SmartDashboard.putNumber("yFeedback", yFeedback);
      SmartDashboard.putNumber("thetaFF", omegaFF);
      SmartDashboard.putNumber("xFF", xFF);
      SmartDashboard.putNumber("yFF", yFF);
    }
    // Drive the robot at the calculated speeds    
    m_robotDrive.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, true);
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

  /**
   * choreoToWPITrajectory
   * Convert a choreo style trajectory to a wpi style trajectory for display purposes only
   * @param choreoTrajectory - Optional<Trajectory<SwerveSample>> 
   * @return wpiTrajectory - Trajectory with only pose filled in.
   */
  public edu.wpi.first.math.trajectory.Trajectory choreoToWPITrajectory(Optional<Trajectory<SwerveSample>> choreoTrajectory) {
    edu.wpi.first.math.trajectory.Trajectory wpiTrajectory = new edu.wpi.first.math.trajectory.Trajectory();
    if (choreoTrajectory.isPresent()) {
      var trajectory = choreoTrajectory.get();
      var samples = trajectory.samples();
      List<edu.wpi.first.math.trajectory.Trajectory.State> states = new ArrayList<>();
      ListIterator<SwerveSample> listIterator = samples.listIterator();
      while (listIterator.hasNext()) {
        var sample = listIterator.next();
        Pose2d pose = new Pose2d(sample.x, sample.y, new Rotation2d(0.0));
        if (isRedAlliance()) {
          pose = FieldConstants.BlueToRedPose(pose);
        }
        edu.wpi.first.math.trajectory.Trajectory.State state = new edu.wpi.first.math.trajectory.Trajectory.State();
        state.poseMeters = pose;
        state.timeSeconds = sample.t;
        states.add(state);
      }
      wpiTrajectory = new edu.wpi.first.math.trajectory.Trajectory(states);
    }
    return wpiTrajectory;
  }

  /*
   * getInitialPose - helper function to return the initial pose of the trajectory if it exists, otherwise return a default pose.
   */
 public Pose2d getInitialPose() {
    return m_initialPose.orElse(new Pose2d());
  }
}
