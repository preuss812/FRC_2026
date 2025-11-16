// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer; // For simulation check and debugging.
import frc.robot.Utilities;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * A command that uses two PID controllers ({@link PIDController}) and a ProfiledPIDController
 * ({@link ProfiledPIDController}) to follow a trajectory {@link Trajectory} with a swerve drive.
 *
 * <p>This command outputs the raw desired Chassis Speeds ({@link ChasisSpeeds}) in an
 * array. The desired wheel and module rotation velocities should be taken from those and used in
 * velocity PIDs.
 *
 * <p>The robot angle controller does not follow the angle given by the trajectory but rather goes
 * to the angle given in the final state of the trajectory.
 *
 * <p>This class is provided by the NewCommands VendorDep
 * 
 * <p>Note: This is a copy of the WPILib command with the addition of features for debug
   * and simulation.
   *
 */
public class PreussSwerveControllerCommand extends Command {
  private final DriveSubsystemSRX m_robotDrive;
  private final PoseEstimatorSubsystem m_poseEstimatorSubsystem;
  private final Timer m_timer = new Timer();
  private Trajectory m_trajectory;
  private final Supplier<Pose2d> m_pose;
  private final HolonomicDriveController m_controller;
  private Supplier<Rotation2d> m_rotationSupplier;
  private final Consumer<ChassisSpeeds> m_driveInputs;
  private double m_count = 0;
  private double m_speedFactor = 1.0; // Can be used to speed up or slow down the path following  1.0 = speed as defined in path.

  /**
   * Constructs a new PreussSwerveControllerCommand2 that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path.
   * This is left to the user to do since it is not appropriate for paths with nonstationary
   * endstates.
   *
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param thetaController The Trajectory Tracker PID controller for angle for the robot.
   * @param desiredRotation The angle that the drivetrain should be facing. This is sampled at each
   *     time step.
   * @param driveInputs The output function to drive the robot [x,y,rot] speeds in meters/sec, meters/sec, and radians/sec
   * @param requirements The subsystems to require.
   */
  public PreussSwerveControllerCommand(
      DriveSubsystemSRX robotDrive,
      PoseEstimatorSubsystem poseEstimatorSubsystem,
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController thetaController,
      Supplier<Rotation2d> desiredRotation,
      Consumer<ChassisSpeeds> driveInputs,
      Subsystem... requirements) {
    this(
        robotDrive,
        poseEstimatorSubsystem,
        trajectory,
        pose,
        new HolonomicDriveController(
            requireNonNullParam(xController, "xController", "PreussSwerveControllerCommand2"),
            requireNonNullParam(yController, "yController", "PreussSwerveControllerCommand2"),
            requireNonNullParam(thetaController, "thetaController", "PreussSwerveControllerCommand2")),
        desiredRotation,
        driveInputs,
        requirements);
  }

  /**
   * Constructs a new PreussSwerveControllerCommand2 that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path.
   * This is left to the user since it is not appropriate for paths with nonstationary endstates.
   *
   * <p>Note 2: The final rotation of the robot will be set to the rotation of the final pose in the
   * trajectory. The robot will not follow the rotations from the poses at each timestep. If
   * alternate rotation behavior is desired, the other constructor with a supplier for rotation
   * should be used.
   *
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param thetaController The Trajectory Tracker PID controller for angle for the robot.
   * @param driveInputs The output function to drive the robot [x,y,rot] speeds in meters/sec, meters/sec, and radians/sec
   * @param requirements The subsystems to require.
   */
  public PreussSwerveControllerCommand(
      DriveSubsystemSRX robotDrive,
      PoseEstimatorSubsystem poseEstimatorSubsystem,
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController thetaController,
      Consumer<ChassisSpeeds> driveInputs,
      Subsystem... requirements) {
    this(
        robotDrive,
        poseEstimatorSubsystem,trajectory,
        pose,
        xController,
        yController,
        thetaController,
        () -> trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation(),
        driveInputs,
        requirements);
  }

  /**
   * Constructs a new PreussSwerveControllerCommand2 that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path-
   * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * <p>Note 2: The final rotation of the robot will be set to the rotation of the final pose in the
   * trajectory. The robot will not follow the rotations from the poses at each timestep. If
   * alternate rotation behavior is desired, the other constructor with a supplier for rotation
   * should be used.
   *
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param controller The HolonomicDriveController for the drivetrain.
   * @param driveInputs The output function to drive the robot [x,y,rot] speeds in meters/sec, meters/sec, and radians/sec
   * @param requirements The subsystems to require.
   */
  public PreussSwerveControllerCommand(
      DriveSubsystemSRX robotDrive,
      PoseEstimatorSubsystem poseEstimatorSubsystem,
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      HolonomicDriveController controller,
      Consumer<ChassisSpeeds> driveInputs,
      Subsystem... requirements) {
    this(
        robotDrive,
        poseEstimatorSubsystem,trajectory,
        pose,
        controller,
        () -> trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation(),
        driveInputs,
        requirements);
  }

  /**
   * Constructs a new PreussSwerveControllerCommand2 that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path-
   * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
   * 
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param controller The HolonomicDriveController for the drivetrain.
   * @param rotationSupplier The angle that the drivetrain should be facing. This is sampled at each
   *     time step.
   * @param driveInputs The output function to drive the robot [x,y,rot] speeds in meters/sec, meters/sec, and radians/sec
   * @param requirements The subsystems to require.
   */
  @SuppressWarnings("this-escape")
  public PreussSwerveControllerCommand(
      DriveSubsystemSRX robotDrive,
      PoseEstimatorSubsystem poseEstimatorSubsystem,
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      HolonomicDriveController controller,
      Supplier<Rotation2d> rotationSupplier,
      Consumer<ChassisSpeeds> driveInputs,
      Subsystem... requirements) {
    m_robotDrive = robotDrive;
    m_poseEstimatorSubsystem = poseEstimatorSubsystem;
    m_trajectory = trajectory; //requireNonNullParam(trajectory, "trajectory", "PreussSwerveControllerCommand2");
    m_pose = requireNonNullParam(pose, "pose", "PreussSwerveControllerCommand2");
    m_controller = requireNonNullParam(controller, "controller", "PreussSwerveControllerCommand2");
    m_rotationSupplier =
      requireNonNullParam(rotationSupplier, "desiredRotation", "PreussSwerveControllerCommand2");

    m_driveInputs =
      requireNonNullParam(driveInputs, "driveInputs", "PreussSwerveControllerCommand2");

    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    if (!RobotContainer.isSimulation()) m_timer.restart();
    m_count = 0;
    if (m_trajectory != null)
      RobotContainer.m_PoseEstimatorSubsystem.field2d.getObject("trajectory").setTrajectory(m_trajectory);
    // Make sure we are wrapping the angles
    m_controller.getThetaController().enableContinuousInput(-Math.PI, Math.PI);
    // Do we need a reset? m_controller.getThetaController().reset(0.0);
  }

  @Override
  public void execute() {
    if (m_trajectory == null) return;

    // get the time (or simulated time)
    double curTime = getTime();

    // get the trajectory information.
    var desiredState = m_trajectory.sample(curTime);

    Pose2d trajectoryPose = desiredState.poseMeters;

    // get the current robot position and orientation.
    Pose2d currentPose = m_pose.get();

    // get the desired rotation for the robot from the supplier and the current rotation
    // and compute the rotation error.
    Rotation2d desiredRotation = m_rotationSupplier.get();
    Rotation2d currentRotation = currentPose.getRotation();
    double m_rotationError = MathUtil.angleModulus(desiredRotation.minus(currentRotation).getRadians());
    
    /*
     * Calculate feedforward velocities (field-relative).
     *The input is a direction stored as the trajectory rotation.
     * Note that the trajectoryPose.rotation is not the robot's intended rotation.
     * The rotation comes from the rotation supplier which for 2025 was typically the 
     * robot rotation which will face the robot's camera at the target april tag.
     * The *FF values are the FeedForward speeds which ignores the actual field coordiantes
     * of the robot and uses the ideal position from the trajectory.
     */
    double xFF = desiredState.velocityMetersPerSecond * trajectoryPose.getRotation().getCos() * m_speedFactor;
    double yFF = desiredState.velocityMetersPerSecond * trajectoryPose.getRotation().getSin() * m_speedFactor;
    double thetaFF = m_controller.getThetaController().calculate(0.0, m_rotationError);

    /*
     * Calculate feedback velocities (based on position error).
     * This provides a correction based on the difference between the rubots actual field position
     * versus the ideal position from the trajectory.  This is what corrects for any error in starting position,
     * slippage, or other factors that would cause the robot to deviate from the intended path.
     */
    double xFeedback = m_controller.getXController().calculate(currentPose.getX(), trajectoryPose.getX());
    double yFeedback = m_controller.getYController().calculate(currentPose.getY(), trajectoryPose.getY());
    
    // Generate the next speeds for the robot
    ChassisSpeeds speeds = new ChassisSpeeds(
      xFF + xFeedback,
      yFF + yFeedback,
      thetaFF
    );

    // Drive the robot at the calculated speeds    
    m_driveInputs.accept( speeds );
    m_count++; // For debugging simulation.
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_driveInputs.accept( new ChassisSpeeds(0.0,0.0,0.0));

    RobotContainer.m_PoseEstimatorSubsystem.field2d.getObject("trajectory").setTrajectory(new Trajectory());
  }

  @Override
  public boolean isFinished() {
    if (m_trajectory == null) return true;
    double curTime = getTime();
  
    return (curTime >=m_trajectory.getTotalTimeSeconds());
  }

  /*
   * getTime - helper function to return time possible modified by the speed factor.
   * m_speedFactor is used to speed up or slow down the path following  1.0 = speed as defined in path.
   * isSimulation check is used to simulate time in a way that allows for breakpoints during simulation.
   * @return time to be used for trajectory sampling.
   */
  private double getTime() {

    double curTime = m_timer.get() * m_speedFactor;  // Slow down time if requested.

    // If we are simulating, increment time based on count of executions.
    // This allows for thinking during breakpoints.  Without this, the time
    // would likely use up the entire time during a breakpoint.
    if (RobotContainer.isSimulation()) {
      curTime = m_count * 0.02 * m_speedFactor; // simulate a 20ms periodic update
    }
    
    return curTime;
  }

  // 
  public void setTrajectory(Trajectory trajectory) {
    m_trajectory = trajectory;
  }
  public PoseEstimatorSubsystem getPoseEstimatorSubsystem() {
    return m_poseEstimatorSubsystem;
  }

  public void setRotationSupplier(Supplier<Rotation2d> rotationSupplier) {
    m_rotationSupplier =  rotationSupplier;
  }

  // The following method is AI generated to reverse a trajectory for red alliance use.
  public Trajectory reverseTrajectory(Trajectory trajectory) {
    int nStates = trajectory.getStates().size();
    Trajectory.State[] reversedStates = new Trajectory.State[nStates];
    for (int i = 0; i < nStates; i++) {
      Trajectory.State state = trajectory.getStates().get(nStates - 1 - i);
      reversedStates[i] = new Trajectory.State(
        state.timeSeconds,
        state.velocityMetersPerSecond,
        state.accelerationMetersPerSecondSq,
        Constants.FieldConstants.BlueToRedPose(state.poseMeters),
        state.curvatureRadPerMeter
      );
    }
    return new Trajectory(java.util.List.of(reversedStates));
  }
  
}
