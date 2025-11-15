// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.AprilTag;
import frc.robot.TrajectoryPlans;
import frc.robot.Utilities;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SwerveToProcessorCommand extends PreussSwerveControllerCommand {

  private AprilTag destination;
  private static Rotation2d robotRotationToFaceProcessor;
  private final boolean faceReef = false;
  private Pose2d aprilTagPose;

  /** Creates a new SwerveToProcessorCommand. */
  public SwerveToProcessorCommand(
    DriveSubsystemSRX robotDrive,
    PoseEstimatorSubsystem poseEstimatorSubsystem
  ) {
    super (
      robotDrive,
      poseEstimatorSubsystem,
      new Trajectory(),
      poseEstimatorSubsystem::getCurrentPose, // Functional interface to feed supplier

      // Position controllers
      new PIDController(AutoConstants.kPXController, 0, 0),
      new PIDController(AutoConstants.kPYController, 0, 0),
      new ProfiledPIDController(
        AutoConstants.kPThetaController,
        0,
        0,
        new TrapezoidProfile.Constraints(Math.PI, Math.PI)
      ),
      //(faceReef)
        //? () -> TrajectoryPlans.reefFacingRotationSupplier(poseEstimatorSubsystem)
      () -> robotRotationToFaceProcessor(),
      robotDrive::driveFieldRelative,
      robotDrive
    );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Trajectory trajectory;
    Pose2d startingPose = getPoseEstimatorSubsystem().getCurrentPose();
    int allianceID = Utilities.getAllianceID();
  
    if (allianceID == FieldConstants.BlueAlliance) {
      destination = VisionConstants.AprilTag.BLUE_PROCESSOR;
    } else {
      destination = VisionConstants.AprilTag.RED_PROCESSOR;
    }

    List<Pose2d> waypoints = new ArrayList<>();
    aprilTagPose = null;
    Pose2d nearTargetPose = null;
    //Pose2d targetPose = null;
    SmartDashboard.putString("TT","Running");
    //commands = new SequentialCommandGroup();

    if (destination == AprilTag.BLUE_PROCESSOR) {
      waypoints = TrajectoryPlans.planTrajectory(TrajectoryPlans.BlueProcessorPlan, startingPose);
      aprilTagPose = getPoseEstimatorSubsystem().getAprilTagPose(AprilTag.BLUE_PROCESSOR.id());
      super.setRotationSupplier(() -> aprilTagPose.getRotation().plus(new Rotation2d(Math.PI)));
      nearTargetPose = DriveConstants.robotFrontAtPose(aprilTagPose, Units.inchesToMeters(12.0));
      robotRotationToFaceProcessor = aprilTagPose.getRotation().plus(new Rotation2d(Math.PI));
      waypoints.add(nearTargetPose);
      //targetPose = Utilities.backToPose(aprilTagPose, 0.5);
    } else if (destination == AprilTag.RED_PROCESSOR) {
      waypoints = TrajectoryPlans.planTrajectory(TrajectoryPlans.RedProcessorPlan, startingPose);
      aprilTagPose = getPoseEstimatorSubsystem().getAprilTagPose(AprilTag.RED_PROCESSOR.id());
      super.setRotationSupplier(() -> aprilTagPose.getRotation().plus(new Rotation2d(Math.PI)));
      nearTargetPose = DriveConstants.robotFrontAtPose(aprilTagPose, Units.inchesToMeters(12.0));
      robotRotationToFaceProcessor = aprilTagPose.getRotation().plus(new Rotation2d(Math.PI));
      waypoints.add(nearTargetPose);
      //targetPose = Utilities.backToPose(aprilTagPose, 0.5);
    } else {
      //targetPose = startingPose; // This will end up doing nothing.
      nearTargetPose = startingPose;
    }
    //if (waypoints.size() > 0)
      Utilities.toSmartDashboard("TT Plan", waypoints);
    
    if (waypoints.size() > 0 && !startingPose.equals(nearTargetPose)) {
        List<Pose2d> sanitizedWaypoints = TrajectoryPlans.sanitizeWaypoints(waypoints);
        Pose2d[] waypointArray = new Pose2d[sanitizedWaypoints.size()];
      
        waypointArray = sanitizedWaypoints.toArray(waypointArray);
        trajectory = TrajectoryPlans.createTrajectory(waypointArray, TrajectoryPlans.m_forwardTrajectoryConfig);
    } else {
        trajectory = null;
    }
    super.setTrajectory(trajectory);
    super.initialize();
  
  }

  public static Rotation2d robotRotationToFaceProcessor() {
    return robotRotationToFaceProcessor;
  }
}
