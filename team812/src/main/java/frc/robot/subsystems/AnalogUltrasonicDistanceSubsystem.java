// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.UltrasonicConstants;

public class AnalogUltrasonicDistanceSubsystem extends SubsystemBase {
  private final AnalogInput ultrasonicSensor;
  private final boolean debug = true;
  public double voltageScaleFactor = 1;

  /** Creates a new UltrasonicDistanceSubsystem. */
  public AnalogUltrasonicDistanceSubsystem() {
    this.ultrasonicSensor = new AnalogInput(UltrasonicConstants.kUltrasonicAnalogPort);
   //                                                 UltrasonicConstants.kUltrasonicFullRange,
     //                                               UltrasonicConstants.kUltrasonicZeroVoltOffset);  

  /* We use the AnalogPotentiometer class for our Analog range sensor model MB1013
   * which is plugged into an analog port (kUltrasonicAnalogPort) on the RoboRio.
   * 
   * MB1013 ranges from 300mm to 50000mm with a 10Hz read rate. Wide detection field. 
   * Resolution of 1-mm
   * Virtually no sensor dead zone, however, large objects closer than 30 cm (300mm) 
   * are typically reported as 30 cm.
   */
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    voltageScaleFactor = 5/RobotController.getVoltage5V();
    if (debug) SmartDashboard.putNumber("US vsf", voltageScaleFactor);
    if (debug) SmartDashboard.putNumber("US cm",getRange());
    //SmartDashboard.putNumber("US voltage", ultrasonicSensor.getAverageVoltage());
    //SmartDashboard.putNumber("US value", ultrasonicSensor.getAverageValue());
  }
  /**
   * return the range to the nearest object to the sensor.
   * @return
   */
  // Get the value from the sensor, scale it by the current Voltage Scale Factor
  // then scale it to centimeters. 
  public double getRange() {
    return ultrasonicSensor.getAverageValue()*voltageScaleFactor*0.125;
  }
}
