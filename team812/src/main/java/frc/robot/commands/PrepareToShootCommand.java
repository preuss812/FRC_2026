// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class PrepareToShootCommand extends Command {
  private final ShooterSubsystem shooter;

  /** Creates a new MotorTest. */
  public PrepareToShootCommand(ShooterSubsystem shooter, PoseEstimatorSubsystem poseEstimator) {
    this.shooter = shooter;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    SmartDashboard.putBoolean("Shooter OK", false);
    SmartDashboard.putBoolean("Feeder OK", false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setShooterMode(ShooterConstants.ShooterMode.AUTO_RANGING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean ignore = readyToShoot();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setShooterMode(ShooterConstants.ShooterMode.IDLE);
    SmartDashboard.putBoolean("Shooter OK", false);
    SmartDashboard.putBoolean("Feeder OK", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean readyToShoot(){
    boolean shooterRPMOkay = shooter.shooterReadyToShoot();
    boolean feederRPMOkay = shooter.feederReadyToShoot();
    SmartDashboard.putNumber("Shooter RPM Error", shooter.shooterError());
    SmartDashboard.putNumber("Feeder RPM Error", shooter.feederError());
    SmartDashboard.putBoolean("Shooter OK", shooterRPMOkay);
    SmartDashboard.putBoolean("Feeder OK", feederRPMOkay);
    return feederRPMOkay && shooterRPMOkay;
  }


}
