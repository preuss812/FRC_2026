// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.BlackBoxSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FlywheelTest extends Command {
  private final FlywheelSubsystem flywheel;
  private final  BlackBoxSubsystem blackBox;
  private final PoseEstimatorSubsystem PoseEstimatorSubsystem;
  private final double[] rangesFromHub = {0, 100, 120, 135, 150, 165, 180};
  /** Creates a new MotorTest. */
  public FlywheelTest(FlywheelSubsystem motor, BlackBoxSubsystem blackBox, PoseEstimatorSubsystem poseEstimator) {
    this.flywheel = motor;
    this.blackBox = blackBox;
    PoseEstimatorSubsystem = poseEstimator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(motor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPos = PoseEstimatorSubsystem.getCurrentPose();
    Translation2d hubPos = new Translation2d(Units.inchesToMeters(181.56), Units.inchesToMeters(158.32));
    //Distance from the center of the robot (Adjust later)
    double shooterOffset = 0;
    double distance = hubPos.getDistance(robotPos.getTranslation()) - shooterOffset;
    double height = Units.inchesToMeters(72);
    double RPM = distanceToRPM(distance);
    flywheel.setRPM(RPM);
    SmartDashboard.putBoolean("RPM OK", canShoot(RPM));

    // double knobPos = blackBox.getPotValue(0);
    // SmartDashboard.putNumber("Black Box Rotation", knobPos);
    // motor.runMotor(knobPos);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public double distanceToRPM(double x){
    //Change coefficents later
    //Maybe: move to Constants.java
    return Constants.FlywheelConstants.a*Math.pow(x, 3) + Constants.FlywheelConstants.b*Math.pow(x,2) + Constants.FlywheelConstants.c*x + Constants.FlywheelConstants.d;
  }

  public boolean canShoot(double targetRPM){
    double actualRPM = flywheel.getRPM();
    return Math.abs(actualRPM-targetRPM) < Constants.FlywheelConstants.RPMTolerance;
  }


}
