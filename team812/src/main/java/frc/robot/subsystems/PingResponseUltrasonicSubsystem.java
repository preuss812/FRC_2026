// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotContainer;
import frc.utils.Line;
import frc.utils.Polygon;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PingResponseUltrasonicSubsystem extends SubsystemBase {
  private final Ultrasonic ultrasonicSensor;
  private final double offsetToBumper;
  private double m_range = 0.0;
  private final boolean debug = false;

  /** Creates a new UltrasonicDistanceSubsystem. */
  public PingResponseUltrasonicSubsystem(int pingChannel, int echoChannel, double offsetToBumper) {
    this.ultrasonicSensor = new Ultrasonic(pingChannel, echoChannel);
    this.offsetToBumper = offsetToBumper;

    // This makes the sensor go out and ping automatically.
    Ultrasonic.setAutomaticMode(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_range = ultrasonicSensor.getRangeMM()/1000.0 - offsetToBumper;
    if (debug) {
      SmartDashboard.putNumber("US cm", getRange()*100.0);
    }
    // TODO: Should we move the getRange call here and save the value rather than having users call getRange)
  }
  
  /**
   * return the range to the nearest object to the sensor.
   * @return
   */
  public double getRange() {
    // Return the range in meters, adjusting for the offset to the bumper.
    return m_range;
  }

  @Override
  public void simulationPeriodic() {
    m_range = simulationRange();
  }

  public double simulationRange() {
    Pose2d currentPose = RobotContainer.m_poseEstimatorSubsystem.getCurrentPose();
    // Compute distance to nearest wall based on current pose.
    double x = currentPose.getX();
    double y = currentPose.getY();
    double distance = -1; // Sentinel value for no wall detected
    Polygon[] obstacles = new Polygon[3];
    // Assuming walls at x=0, x=FieldWidth, y=0, y=FieldHeight
    
    Polygon fieldBoundary = new Polygon( Arrays.asList(
      new Translation2d(0,0),
      new Translation2d(0,FieldConstants.fieldWidth),
      new Translation2d(FieldConstants.fieldLength, FieldConstants.fieldWidth),
      new Translation2d(FieldConstants.fieldLength,0)
    ));
    obstacles[0] = fieldBoundary;
    Polygon blueHub = new Polygon( Arrays.asList(
      new Translation2d(3,3.5),
      new Translation2d(3,4.5),
      new Translation2d(4, 5),
      new Translation2d(5,4.5),
      new Translation2d(5,3.5),
      new Translation2d(4,3)
    ));
    obstacles[1] = blueHub;
    Polygon redHub = new Polygon( Arrays.asList(
      new Translation2d(11,3.5),
      new Translation2d(11,4.5),
      new Translation2d(12, 5),
      new Translation2d(13,4.5),
      new Translation2d(13,3.5),
      new Translation2d(12.5,3)
    ));
    obstacles[2] = redHub;

    //obstacles[0] = new Line(new Translation2d(0,0), 0);
    //obstacles[1] = new Line(new Translation2d(0,0), Double.NaN);
    //obstacles[2] = new Line(new Translation2d(0,FieldConstants.fieldWidth), 0);
    //obstacles[3] = new Line(new Translation2d(FieldConstants.fieldLength,0), Double.NaN);

    Line robotVector = new Line(currentPose.getTranslation(), currentPose.getRotation().getTan());

    for (int i = 0; i < obstacles.length; i++) {
      List<Translation2d> intersections = obstacles[i].intersections(robotVector);
      for (Translation2d intersection : intersections) {
        if (intersection != null) {
          double intersectX = intersection.getX();
          double intersectY = intersection.getY();
          // Check if intersection is in front of the robot
          double dx = intersectX - x;
          double dy = intersectY - y;
          double angleToIntersection = Math.atan2(dy, dx);
          double robotAngle = currentPose.getRotation().getRadians();
          double angleDiff = Math.abs(angleToIntersection - robotAngle);
          if (angleDiff < Math.PI / 2) { // within 90 degrees
            double distToIntersection = Math.hypot(dx, dy) - DriveConstants.robotCenterToFrontBumper.getX();
            if (distance == -1 || distToIntersection < distance) {
              distance = distToIntersection;
            }
          }
        }
      }
    }
    return distance;
  }
}
