// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.AllianceConfigurationSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrepareToShootCommand extends Command {
  private final ShooterSubsystem shooter;
  private final FeederSubsystem feeder;
  private final PoseEstimatorSubsystem PoseEstimatorSubsystem;
  /** Creates a new MotorTest. */
  public PrepareToShootCommand(ShooterSubsystem shooter, FeederSubsystem feeder, PoseEstimatorSubsystem poseEstimator) {
    this.shooter = shooter;
    this.feeder = feeder;
    PoseEstimatorSubsystem = poseEstimator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = PoseEstimatorSubsystem.getCurrentPose();
    Translation2d hubPos = AllianceConfigurationSubsystem.getHubCenter();
    //Distance from the center of the robot (Adjust later)
    double shooterOffset = 0;
    double distance = robotPose.getTranslation().getDistance(hubPos) - shooterOffset;
    double RPM = distanceToRPM(distance);
    shooter.setRPM(RPM);
    feeder.setRPM(RPM);
    //shooter.runMotor(RPM*ShooterConstants.RPMToVolts/12.0);
    SmartDashboard.putBoolean("RPM OK", canShoot(RPM));

    

    // double knobPos = blackBox.getPotValue(0);
    // SmartDashboard.putNumber("Black Box Rotation", knobPos);
    // motor.runMotor(knobPos);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.runMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public double distanceToRPM(double x){
    //Change coefficents later
    return ShooterConstants.a*Math.pow(x, 3) + ShooterConstants.b*Math.pow(x,2) + ShooterConstants.c*x + ShooterConstants.d;
  }

  public boolean canShoot(double targetRPM){
    double actualRPM = shooter.getRPM();
    return Math.abs(actualRPM-targetRPM) < ShooterConstants.RPMTolerance;
  }


}
