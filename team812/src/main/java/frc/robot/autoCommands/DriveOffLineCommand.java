// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveWithoutVisionCommand;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveOffLineCommand extends SequentialCommandGroup {
  /** Creates a new DriveOffLineCommand. */
  public DriveOffLineCommand() {
    addCommands(
      // Backwards 1 meter.  This should be enough to get off the starting line and into the area where we can see the april tags.
      new DriveWithoutVisionCommand(
        RobotContainer.m_robotDrive, 
        RobotContainer.m_poseEstimatorSubsystem,
        new Pose2d(-1.0, 0, new Rotation2d(0.0)),
         RobotContainer.m_robotDrive.defaultAutoConfig ),
      new InstantCommand(() -> RobotContainer.m_ShooterSubsystem.setRPM(3000.0)) // Start the flywheel spinning at an initial guess at the required rpm.  Could do better - TODO
    );
  }

}
