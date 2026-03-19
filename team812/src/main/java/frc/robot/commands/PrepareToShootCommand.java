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
import frc.robot.RobotContainer;
import frc.robot.subsystems.AllianceConfigurationSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class PrepareToShootCommand extends Command {
  private final ShooterSubsystem shooter;
  private final FeederSubsystem feeder;
  private final PoseEstimatorSubsystem PoseEstimatorSubsystem;
  private final double pwl[] = { // array or rpm vs feet from hub
    2500.0, // 0
    2500.0, // 1
    2500.0, // 2
    2500.0, // 3
    2500.0, // 4
    2500.0, // 5
    2500.0, // 6
    2550.0, // 7
    2600.0, // 8
    2695.0, // 9
    2790.0, // 10
    2875.0, // 11
    2970.0, // 12
    3090.0, // 13
    3200.0, // 14
    3300.0, // 15
    3400.0, // 16
    3500.0, // 17
    3600.0, // 18
    3700.0, // 19
    3800.0, // 20
  };
  ExponentialSmoother exponentialSmoother = new ExponentialSmoother(1.0);

  public class ExponentialSmoother {
    private double alpha = 0.0; // weight of the new sample. (1-alpha is the weight for the old samples)
    private boolean reset = true;
    private double value = 0.0;
    public ExponentialSmoother(double alpha) {
      this.alpha = alpha;
      this.reset = true;
    }

    public void reset() {
      reset = true;
    }

    public double addSample(double x) {
      if (reset) {
        value = x;
        reset = false;
      } else {
        value = x * alpha + value * (1.0 - alpha);
      }
      return value;
    }
  }

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
    exponentialSmoother.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = PoseEstimatorSubsystem.getCurrentPose();
    Translation2d hubPos = AllianceConfigurationSubsystem.getHubCenter();
    //Distance from the center of the robot (Adjust later)
    double shooterOffset = 0;
    double distance = robotPose.getTranslation().getDistance(hubPos) - shooterOffset;
    double correction = RobotContainer.getShooterCorrection();
    SmartDashboard.putString("Distance to Hub", String.format("%1dft %1din", (int)Units.metersToInches(distance)/12, (int)Units.metersToInches(distance)%12));
    double adjustedDistance = exponentialSmoother.addSample(distance+correction);
    double RPM = distanceToRPMPWL(MathUtil.clamp(adjustedDistance, 1.0, 6.0)); // The cubic is not fit beyond this range.
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

  public double distanceToRPMPWL(double x) {
    double feet = Units.metersToFeet(x);
    int wholeFeet = (int)Math.floor(Units.metersToFeet(Math.abs(x)));
    double remainder = feet - wholeFeet;
    if (wholeFeet > pwl.length - 2)
      wholeFeet = pwl.length - 2;
    return pwl[wholeFeet] + (pwl[wholeFeet+1] - pwl[wholeFeet]) * remainder;
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
