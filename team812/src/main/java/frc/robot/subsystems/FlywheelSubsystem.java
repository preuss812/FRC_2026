// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlywheelSubsystem extends SubsystemBase {
  /** Creates a new FlywheelSubsystem. */

  private final TalonSRX flywheel;

    // Shooter settings
    //private final double maxRPM = 5320; // For a CIM motor. 
    private final double defaultRPM = 3000;
    private final double defaultRPMFeedForward = 1.0; // TODO: need a real number.
    private double targetRPM = defaultRPM;  // desired shooter wheel speed
    private double currentRPM; // the current rpm of the flywheel.
    // native units are ticks per 100ms.
    // the revrobotics through bore encoder I think has 4096 ticks per revolution.
    // Therefore rpm to ticks is 1 rpm * (4096 ticks/rpm) * (1 minute/60 seconds) * (0.1 second/100ms) = rpm to native
    private final double RPMToNativeUnits = 1.0 * (4096.0) * (1.0/60.0) * (0.1);
    private final double nativeUnitsToRPM = 1.0/RPMToNativeUnits; // Typical CTRE Mag Encoder CPR

    private final int pidIdx = 0;
    private final int slotIdx = 0;
    private final int timeoutMs = 10;
    

  public FlywheelSubsystem(int flywheelCANId) {
    flywheel = new TalonSRX(flywheelCANId);
    flywheel.configFactoryDefault(); // set to default settings (ie a known starting state)
    // encoder = new CANcoder(encoderCANId); // Would be needed if the encoder was not going direclty to the Talon.
    // Configure feedback sensor
    flywheel.configSelectedFeedbackSensor(
      com.ctre.phoenix.motorcontrol.FeedbackDevice.CTRE_MagEncoder_Relative, pidIdx, timeoutMs
    );
    // Inversion settings (adjust based on your robot)
    flywheel.setInverted(true); // TODO verify
    flywheel.setSensorPhase(true);// TODO verify
    // PID settings — TODO: tune these!
    flywheel.config_kF(slotIdx, rpmToFeedForward(targetRPM), timeoutMs); // Probably should be set when target speed is set.
    flywheel.config_kP(slotIdx, 0.15, timeoutMs);
    flywheel.config_kI(slotIdx, 0.0, timeoutMs);
    flywheel.config_kD(slotIdx, 2.0, timeoutMs);
    double targetVelocity_UnitsPer100ms = rpmToNativeUnits(targetRPM);
    flywheel.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // closed-loop velocity control
    currentRPM = nativeUnitsToRPM(flywheel.getSelectedSensorVelocity(pidIdx));
    // telemetry
    SmartDashboard.putNumber("Flywheel RPM", currentRPM);
  }
  /** Convert RPM to Talon SRX velocity units (ticks per 100ms) */
  private double rpmToNativeUnits(double rpm) {
      return rpm / nativeUnitsToRPM;
  }

  /** Convert sensor velocity to real RPM */
  private double nativeUnitsToRPM(double sensorUnitsPer100ms) {
      return sensorUnitsPer100ms * nativeUnitsToRPM;
  }

  public void runMotor(double pOut){
    flywheel.set(ControlMode.PercentOutput, pOut);
  }

  /**
   * setRPM - set the target rpm
   * @param rpm (double) The rpm target for the flywheel
   */
  public void setRPM(double rpm) {
    targetRPM = rpm;
    double targetVelocity = rpmToNativeUnits(rpm);
    flywheel.config_kF(slotIdx, rpmToFeedForward(rpm), timeoutMs);
    flywheel.set(ControlMode.Velocity, targetVelocity);
    SmartDashboard.putNumber("Flywheel Target RPM", targetRPM);
  }

  public double getRPM() {
    return currentRPM;
  }

  public double rpmToFeedForward(double rpm) { 
     //TODO: find a good value/equation for the conversion.
    double feedForward = rpm / defaultRPM * defaultRPMFeedForward;
    return feedForward;
  }

  public void stop() {
    setRPM(0);
  }

}