// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PingResponseUltrasonicSubsystem extends SubsystemBase {
  private final Ultrasonic ultrasonicSensor;
  private final double offsetToBumper;
  private final boolean debug = true;

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
    return ultrasonicSensor.getRangeMM()/1000.0 - offsetToBumper;
  }
}
