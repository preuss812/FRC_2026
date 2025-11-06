// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.TrajectoryPlans;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SwerveToProcessorCommand extends SequentialCommandGroup {
  /** Creates a new SwerveToProcessorCommand. */
  public SwerveToProcessorCommand(
    DriveSubsystemSRX robotDrive,
    PoseEstimatorSubsystem poseEstimatorSubsystem
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    var thetaController = new ProfiledPIDController(
    AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    boolean faceReef = true;
    Supplier<Rotation2d> rotationSupplier =  (faceReef) 
      ? () -> TrajectoryPlans.reefFacingRotationSupplier(poseEstimatorSubsystem)
      : () -> SwerveToProcessorSetupCommand.robotRotationToFaceProcessor();
    PreussSwerveControllerCommand debugSwerveControllerCommand = new PreussSwerveControllerCommand(
      null,
      poseEstimatorSubsystem::getCurrentPose, // Functional interface to feed supplier
      DriveConstants.kDriveKinematics,

      // Position controllers
      new PIDController(AutoConstants.kPXController, 0, 0),
      new PIDController(AutoConstants.kPYController, 0, 0),
      thetaController,
      rotationSupplier,
      robotDrive::setModuleStates,
      robotDrive);

    addCommands(
      new SwerveToProcessorSetupCommand(robotDrive, poseEstimatorSubsystem, debugSwerveControllerCommand)
      , debugSwerveControllerCommand
    );
  }
}
