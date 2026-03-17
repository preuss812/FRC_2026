// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederConstants;
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
    SmartDashboard.putString("Distance to Hub", String.format("%1dft %1din", (int)Units.metersToInches(distance)/12, (int)Units.metersToInches(distance)%12));
    double RPM = distanceToRPM(MathUtil.clamp(distance, 1.0, 6.0)); // The cubic is not fit beyond this range.
    shooter.setRPM(RPM);
    feeder.setRPM(RPM);
    SmartDashboard.putBoolean("Shooter Ready", readyToShoot(RPM));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    feeder.stop();
    SmartDashboard.putBoolean("Shooter Ready", false);
    SmartDashboard.putNumber("Shooter RPM Error", -1); // -1 is a sentinel value indicating we are open loop.
    SmartDashboard.putBoolean("Shooter RPM OK", false);
    SmartDashboard.putNumber("Feeder RPM Error", -1); // -1 is a sentinel value indicating we are open loop.
    SmartDashboard.putBoolean("Feeder RPM OK", false);
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

  public boolean readyToShoot(double targetRPM){
    double shooterRPM = shooter.getRPM();
    double feederRPM = feeder.getRPM();
    boolean shooterRPMOkay = Math.abs(shooterRPM-targetRPM) < ShooterConstants.RPMTolerance;
    boolean feederRPMOkay = Math.abs(feederRPM-targetRPM) < FeederConstants.RPMTolerance;
    SmartDashboard.putNumber("Shooter RPM Error", shooterRPM - targetRPM);
    SmartDashboard.putNumber("Feeder RPM Error", feederRPM - targetRPM);
    SmartDashboard.putBoolean("Shooter RPM OK", shooterRPMOkay);
    SmartDashboard.putBoolean("Feeder RPM OK", feederRPMOkay);
    return feederRPMOkay && shooterRPMOkay;
  }


}
