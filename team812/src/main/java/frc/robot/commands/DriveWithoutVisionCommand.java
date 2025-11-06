// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utilities;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.utils.DrivingConfig;
import frc.utils.PreussAutoDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveWithoutVisionCommand extends Command {
  private final DriveSubsystemSRX robotDrive;
  private final Pose2d relativeMove;
  private final PreussAutoDrive autoDrive;
  private final DrivingConfig config;
  private Pose2d targetPose;
  private boolean onTarget = true;
  private boolean debug = false;
  
  /** Creates a new DriveWithoutVisionCommand. */
  public DriveWithoutVisionCommand(
    DriveSubsystemSRX robotDrive
  , PoseEstimatorSubsystem poseEstimatorSubsystem
  , DrivingConfig config
  , Pose2d pose
  ) {
    this.robotDrive = robotDrive;
    this.relativeMove = pose;
    this.config = config == null ? robotDrive.defaultAutoConfig : config;
    autoDrive = new PreussAutoDrive(robotDrive,  poseEstimatorSubsystem, this.config);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    onTarget = false;
    autoDrive.reset();

    Pose2d initialPose = robotDrive.getPose();
    if (Utilities.isBlueAlliance()) {
      targetPose = new Pose2d(
        initialPose.getX() + relativeMove.getX()
        , initialPose.getX() + relativeMove.getY()
        , new Rotation2d(MathUtil.angleModulus(initialPose.getRotation().getRadians()+relativeMove.getRotation().getRadians())));

    } else {
      targetPose = new Pose2d(
        initialPose.getX() - relativeMove.getX(),
        initialPose.getY() - relativeMove.getY(),
        initialPose.getRotation().rotateBy(relativeMove.getRotation())
      );
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (onTarget) return; // Prevent code from running if we are there or if there was no apriltag.

    Translation2d translationErrorToTarget;
    double rotationError;
    Pose2d robotPose;
    double xSpeed = 0.0;
    double ySpeed = 0.0;
    double rotationSpeed = 0.0;

    // get the robot's position on the field.
    robotPose = robotDrive.getPose();

    // Calculate the X and Y offsets to the target location
    //translationErrorToTarget = new Translation2d( targetPose.getX() - robotPose.getX(), targetPose.getY() - robotPose.getY());
    translationErrorToTarget = new Translation2d( robotPose.getX() - targetPose.getX(), robotPose.getY() - targetPose.getY());

    double desiredRotation = targetPose.getRotation().getRadians();
    rotationError = MathUtil.angleModulus(MathUtil.angleModulus(robotPose.getRotation().getRadians()) - desiredRotation);

    // Make sure the rotation error is between -PI and PI
    if (debug) SmartDashboard.putNumber("AutoDrive R", Units.radiansToDegrees(rotationError));
    if (debug) SmartDashboard.putNumber("AutoDrive X", translationErrorToTarget.getX());
    if (debug) SmartDashboard.putNumber("AutoDrive Y", translationErrorToTarget.getY());
    
    // Test to see if we have arrived at the requested pose within the specified toleranes
    if (Math.abs(translationErrorToTarget.getX()) < config.getLinearTolerance()
    &&  Math.abs(translationErrorToTarget.getY()) < config.getLinearTolerance()
    &&  (Math.abs(rotationError) < config.getAngularTolerance())) {
      // We are close enough.  Stop the robot and the command.
      if (debug) SmartDashboard.putBoolean("AutoDrive OnTarget", true);
      xSpeed = 0.0;
      ySpeed = 0.0;
      rotationSpeed = 0.0;
      onTarget = true;
    } else {
      // We are not close enough yet./

      // Transform the error into targetPose coordinates
      Translation2d targetPoseErrorVector = translationErrorToTarget.rotateBy(targetPose.getRotation()); //  vector from apriltag to the robot rotated in april tag space
      //Translation2d targetPoseErrorVector = translationErrorToTarget.rotateBy(new Rotation2d(headingToTargetPose)); //  vector from apriltag to the robot rotated in april tag space

      // Calculate the speeds in the coordinates system defined by the april tag
      // It is important that clipping occurs here and not below as clipping in the field coordinates 
      // will lead to paths that may initially veer away from the target.
      double xSpeedTargetPose = autoDrive.calculateX(targetPoseErrorVector.getX());
      double ySpeedTargetPose = autoDrive.calculateY(targetPoseErrorVector.getY());
      
      // Enforce maThrottle by scaling the magnitude of the error vector.
      // If we clamped instead we would see a slightly off initial angle when clipping happens.
      if (Math.sqrt(xSpeedTargetPose*xSpeedTargetPose + ySpeedTargetPose*ySpeedTargetPose) > config.getMaxThrottle()) {
        double scaleFactor =  config.getMaxThrottle() / Math.sqrt(xSpeedTargetPose*xSpeedTargetPose + ySpeedTargetPose*ySpeedTargetPose);
        xSpeedTargetPose *= scaleFactor;
        ySpeedTargetPose *= scaleFactor;
      }

      // Rotate the calculated speeds back to field coordinates.
      Translation2d targetPoseSpeeds = new Translation2d(xSpeedTargetPose,ySpeedTargetPose);
      Translation2d unrotatedSpeedsPose = targetPoseSpeeds.rotateBy(targetPose.getRotation().times(-1));
      
      // Use the unrotated speeds to control the robot.  It is possible that these speeds could exceed te max throttle but dont
      // clip them unless absolutely necessary to avoid artifacts in the paths.
      // The extra magnitude is more or less limited to sqrt(2)*maxThrottle
      xSpeed = MathUtil.clamp(unrotatedSpeedsPose.getX(), -1.0, 1.0); // Not reclamping because we did so aove with scaleFactor;
      ySpeed = MathUtil.clamp(unrotatedSpeedsPose.getY(), -1.0, 1.0);

      // We are controlling rotation whether we use it for "onTarget" calculations or not.
      rotationSpeed = autoDrive.calculateClampedRotation(rotationError);
    }
    if (debug) SmartDashboard.putNumber("AutoDrive xSpeed", xSpeed);
    if (debug) SmartDashboard.putNumber("AutoDrive ySpeed", ySpeed);
    if (debug) SmartDashboard.putNumber("AutoDrive rSpeed", rotationSpeed);
    autoDrive.drive(xSpeed, ySpeed, rotationSpeed, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    autoDrive.drive(0, 0, 0, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return onTarget;
  }
} // GotoPoseCommand class
