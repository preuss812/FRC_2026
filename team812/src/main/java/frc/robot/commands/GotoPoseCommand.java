// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.utils.DrivingConfig;
import frc.utils.PreussAutoDrive;
import frc.robot.Constants.VisionConstants;
import frc.robot.Utilities;

public class GotoPoseCommand extends Command {
  
  /** Creates a new command to move the robot to the specified pose. */
  protected final DriveSubsystemSRX robotDrive;
  protected final PoseEstimatorSubsystem poseEstimatorSubsystem;
  protected Pose2d targetPose;
  protected final boolean driveFacingFinalPose;
  final DrivingConfig config;
  protected final PreussAutoDrive autoDrive;
  ;
  protected boolean onTarget;
  private boolean controlRotation = true; // This is a holdover from when controlRotation was not working well.
  private boolean debug = true; // turn on/off SmartDashBoard feedback
  
  /**
   * Drive to the specified distance from the best april tag currently in view.
   * @params poseEstimatorSubsystem - the pose estimator subsystem
   * @params robotDrive - the DriveSubsystemSRX drivetrain to drive the robot
   * @params targetPose - the target location and orientation for the robot.
   */
  public GotoPoseCommand(
      DriveSubsystemSRX robotDrive
    , PoseEstimatorSubsystem poseEstimatorSubsystem
    , Pose2d targetPose
    , boolean driveFacingFinalPose
    , DrivingConfig config
     ) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    this.robotDrive = robotDrive;
    this.poseEstimatorSubsystem = poseEstimatorSubsystem;
    this.targetPose = targetPose;
    this.driveFacingFinalPose = driveFacingFinalPose;
    this.config = config == null ? robotDrive.defaultAutoConfig : config;

    this.autoDrive = new PreussAutoDrive(robotDrive, poseEstimatorSubsystem, this.config);
    onTarget = false;

    addRequirements(robotDrive, poseEstimatorSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    onTarget = true; // Not really but if we dont find a target, this will cause the command to end immediately.
    if (targetPose == null) return;
    
    // Reset the pid controllers
    autoDrive.reset();
    if (debug) Utilities.toSmartDashboard("AutoDrive T", targetPose);

    onTarget = false; // Defer this calculation to this.isFinished()
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
    robotPose = autoDrive.getCurrentPose();

    // Calculate the X and Y offsets to the target location
    //translationErrorToTarget = new Translation2d( targetPose.getX() - robotPose.getX(), targetPose.getY() - robotPose.getY());
    translationErrorToTarget = new Translation2d( robotPose.getX() - targetPose.getX(), robotPose.getY() - targetPose.getY());

    // Control rotation to always face directly at the target as it approaches.
    // This is to ensure that we get the most updates to our pose estimator for best possitioning accuracy.
    // Calculate the tangent of the translation error.
    double desiredRotation;
    double errorVectorMagnitude = Math.pow(Math.pow(translationErrorToTarget.getX(),2) + Math.pow(translationErrorToTarget.getY(),2),0.5);

    // If we are close to the target or not tracking an apriltag, rotate to the final pose.
    if (errorVectorMagnitude  < 1.0 /* meters */ || !driveFacingFinalPose) {
      desiredRotation = targetPose.getRotation().getRadians();
    } else {
      desiredRotation = MathUtil.angleModulus(Utilities.getHeading(robotPose.getTranslation(), targetPose.getTranslation()))+VisionConstants.rearCameraHeading;
    }
    
    rotationError = MathUtil.angleModulus(MathUtil.angleModulus(robotPose.getRotation().getRadians()) - desiredRotation);

    // Make sure the rotation error is between -PI and PI
    if (debug) SmartDashboard.putNumber("AutoDrive R", Units.radiansToDegrees(rotationError));
    if (debug) SmartDashboard.putNumber("AutoDrive X", translationErrorToTarget.getX());
    if (debug) SmartDashboard.putNumber("AutoDrive Y", translationErrorToTarget.getY());
    
    // Test to see if we have arrived at the requested pose within the specified toleranes
    if (Math.abs(translationErrorToTarget.getX()) < config.getLinearTolerance()
    &&  Math.abs(translationErrorToTarget.getY()) < config.getLinearTolerance()
    &&  ((!controlRotation) || (Math.abs(rotationError) < config.getAngularTolerance()))) {
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
