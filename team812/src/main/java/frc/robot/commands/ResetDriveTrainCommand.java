// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.RobotContainer;

public class ResetDriveTrainCommand extends SequentialCommandGroup {
  /** Creates a new ResetDriveTrainCommand. */
  public ResetDriveTrainCommand() {
    // reset the robot coordinates to have forward be the direction the robot is facing.
    // Put the robot in slow/precision driving mode.
    addCommands(
      new SequentialCommandGroup(
        new StopAllMotorsCommand(),
        //new ArmHomeCommand(RobotContainer.m_ArmRotationSubsystem),
        new InstantCommand(()->RobotContainer.setGyroAngleToStartMatch()),
        new InstantCommand(()->RobotContainer.m_robotDrive.setDrivingMode(DriveSubsystemSRX.DrivingMode.PRECISION), RobotContainer.m_robotDrive)
      )
    );
  }
}
