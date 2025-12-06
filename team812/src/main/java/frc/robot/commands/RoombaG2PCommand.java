// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PingResponseUltrasonicSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.utils.DrivingConfig;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RoombaG2PCommand extends GotoPoseCommand {

  /** Creates a new command to move the robot to the specified pose. */
  private PingResponseUltrasonicSubsystem m_pingResponseUltrasonicSubsystem;
  private Pose2d targetPose;
  private enum State{ SEARCHING, BACKINGUP, ROTATING};
  private State currentState = State.SEARCHING;
  private boolean debug = true; // turn on/off SmartDashBoard feedback

  /** Creates a new RombaG2PCommand. */
  public RoombaG2PCommand(
    DriveSubsystemSRX robotDrive
    , PoseEstimatorSubsystem poseEstimatorSubsystem
    , PingResponseUltrasonicSubsystem pingResponseUltrasonicSubsystem
    , DrivingConfig config
  ) {

    super(robotDrive, poseEstimatorSubsystem, new Pose2d(), config);
    m_pingResponseUltrasonicSubsystem = pingResponseUltrasonicSubsystem;


    // Use addRequirements() here to declare subsystem dependencies.
    // super adds the requirements.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Get the current field position.
    Pose2d currentPose = super.getPoseEstimatorSubsystem().getCurrentPose();

    // Set the target to be 2 meters away in the direction the robot is currently pointing.

    targetPose = new Pose2d(
      currentPose.getX()+Math.cos(currentPose.getRotation().getRadians()*2.0),
      currentPose.getY()+Math.sin(currentPose.getRotation().getRadians()*2.0),
      currentPose.getRotation()
    );
    setTargetPose(targetPose);
    currentState = State.SEARCHING;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Check progress and possibly reset the target position if we are near the target 
    // or have encountered an obstacle
    Pose2d currentPose = super.getPoseEstimatorSubsystem().getCurrentPose();
    double distanceMeters = m_pingResponseUltrasonicSubsystem.getRange();
    SmartDashboard.putNumber("USM", distanceMeters);
    double maxSearchDistanceMeters = 16.0; // Meters
    if (distanceMeters < 0.0) {
      distanceMeters = maxSearchDistanceMeters + 1;
    }
    boolean onTarget = 
        Math.abs(currentPose.getX() - targetPose.getX()) < getConfig().getLinearTolerance()
      && Math.abs(currentPose.getX() - targetPose.getX()) < getConfig().getLinearTolerance() 
      && Math.abs(currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians()) < getConfig().getAngularTolerance();

    if (currentState == State.SEARCHING)
    {
      if (distanceMeters <= 0.3) // Obstacle detected within 30 cm
      {
        currentState = State.BACKINGUP;
        if (debug) SmartDashboard.putString("RombaG2P", "Backing Up");
        targetPose = new Pose2d(
          currentPose.getX()-0.2*Math.cos(currentPose.getRotation().getRadians()),
          currentPose.getY()-0.2*Math.sin(currentPose.getRotation().getRadians()),
          currentPose.getRotation()
        );
        super.setTargetPose(targetPose);
      }
      else if (onTarget)
      {
        currentState = State.ROTATING;
        if (debug) SmartDashboard.putString("RombaG2P", "Rotating");
        targetPose = new Pose2d(
          currentPose.getX(),
          currentPose.getY(),
          currentPose.getRotation().rotateBy(new Rotation2d(Math.PI/2.0))
        );
        super.setTargetPose(targetPose);

      }
    } else if (currentState == State.BACKINGUP) {
      if (onTarget) {
        currentState = State.ROTATING;
        if (debug) SmartDashboard.putString("RombaG2P", "Rotating");
        targetPose = new Pose2d(
          currentPose.getX(),
          currentPose.getY(),
          currentPose.getRotation().rotateBy(new Rotation2d(Math.PI/2.0))
        );
        super.setTargetPose(targetPose);
      }
    } else { // currentState == State.ROTATING
      if (onTarget) {
        // Finished rotating but still see an obstacle? Then rotate again.
        if (distanceMeters <= 0.3) {
          currentState = State.ROTATING;
          if (debug) SmartDashboard.putString("RombaG2P", "Rotating");
          targetPose = new Pose2d(
            currentPose.getX(),
            currentPose.getY(),
            currentPose.getRotation().rotateBy(new Rotation2d(Math.PI))
          );
          super.setTargetPose(targetPose);
        } else {
          // No obstacle, so go search again.
          currentState = State.SEARCHING;
          if (debug) SmartDashboard.putString("RombaG2P", "Searching");
          targetPose = new Pose2d(
            currentPose.getX()+maxSearchDistanceMeters*Math.cos(currentPose.getRotation().getRadians()),
            currentPose.getY()+maxSearchDistanceMeters*Math.sin(currentPose.getRotation().getRadians()),
            currentPose.getRotation()
          );
          super.setTargetPose(targetPose);
        }
      }
    }
    super.execute();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // Never finished.  Just keep randombly driving.
  }
}
