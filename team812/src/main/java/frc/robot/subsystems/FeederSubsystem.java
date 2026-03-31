// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {

    // Feeder settings
    private double targetRPM = 0.0;  // desired feeder wheel speed
    private double currentRPM; // the current rpm of the feeder.
    // native units are rpm for the Spark MAX closed loop controller.
    // With external through bore encoder this calculation would be more typical:
    //  RPMToNativeUnits = 1 rpm * (4096 ticks/rpm) * (1 minute/60 seconds) * (0.1 second/100ms)
    private final double RPMToNativeUnits = 1.0;
    private final double nativeUnitsToRPM = 1.0/RPMToNativeUnits; // Typical CTRE Mag Encoder CPR
    private final SparkFlex motor1;
    private final SparkFlexSim motor1Sim;
    private final DCMotor m_dcMotor = DCMotor.getNEO(1);

    //private final SparkFlex motor2;
    private final SparkFlexConfig motorConfig;
    //private final SparkFlexConfig motorFollowerConfig;
    private final SparkClosedLoopController closedLoopController;
    private final RelativeEncoder encoder;
    
    private boolean debug = false;

  public FeederSubsystem(int feederCANId) {
      /** Creates a new FeederSubsystem. */

  /*
     * Initialize the SPARK MAX and get its encoder and closed loop controller
     * objects for later use.
     */
    motor1 = new SparkFlex(CANConstants.kFeederMotor, MotorType.kBrushless);
    motor1Sim = new SparkFlexSim(motor1, m_dcMotor);

    //motor2 = new SparkFlex(CANConstants.kFeederMotor2, MotorType.kBrushless);
    closedLoopController = motor1.getClosedLoopController();
    encoder = motor1.getEncoder();

    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    motorConfig = new SparkFlexConfig();
    //motorFollowerConfig = new SparkFlexConfig();
    motorConfig.inverted(false);
    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    motorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);
    /*
     * Configure the 2nd motor on the Feeder to follow, inverted, the primary motor
     */
    //motorFollowerConfig
    //  .apply(motorConfig)
    //  .follow(motor1)
    //  .inverted(true);

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for velocity control in slot 1
        .p(FeederConstants.kP)
        .i(FeederConstants.kI)
        .d(FeederConstants.kD)
        .outputRange(FeederConstants.minOutputPercent, FeederConstants.maxOutputPercent)
        .feedForward
          // kV is now in Volts, so we multiply by the nominal voltage (12V)
          .kV(FeederConstants.kV);

    /*
     * Apply the configuration to the SPARK MAX.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    motor1.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    /*
    motorFollowerConfig
      .apply(motorConfig)
      .follow(motor1)
      .inverted(true);
    motor2.configure(motorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    */

    // Initialize dashboard values
    //SmartDashboard.setDefaultNumber("Feeder RPM Target", 0);
    currentRPM = 0;
    //SmartDashboard.putBoolean("Feeder OK", true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //double rpm = SmartDashboard.getNumber("Feeder RPM Target", 0.0);
    //setRPM(rpm);
    // closed-loop velocity control
    currentRPM = encoder.getVelocity();
    // telemetry
    SmartDashboard.putNumber("Feeder RPM", currentRPM);
    
    if (debug) {
      double current = motor1.getOutputCurrent();
      double appliedOutput = motor1.getAppliedOutput();
      SmartDashboard.putNumber("feeder I", current);
      SmartDashboard.putNumber("feeder V", appliedOutput);
    }
  }

  public void runMotor(double pOut){
        SmartDashboard.putString("FeederMode", "PERCENT");

    pOut = MathUtil.clamp(pOut, FeederConstants.minOutputPercent, FeederConstants.maxOutputPercent);
    closedLoopController.setSetpoint(pOut, ControlType.kDutyCycle);
  }

  public double rpmToNativeUnits(double rpm) {
    return RPMToNativeUnits * rpm;
  }

  /**
   * setRPM - set the target rpm
   * @param rpm (double) The rpm target for the feeder
   */
  public void setRPM(double rpm) {
    SmartDashboard.putString("FeederMode", "RPM");
    targetRPM = rpm;
    double targetVelocity = rpmToNativeUnits(rpm * 0.7);
    closedLoopController.setSetpoint(targetVelocity, ControlType.kVelocity);
    SmartDashboard.putNumber("Feeder Target RPM", targetRPM);
  }

  public double getRPM() {
    return targetRPM * nativeUnitsToRPM;
  }

  public void stop() {
    SmartDashboard.putString("FeederMode", "STOP");

    closedLoopController.setSetpoint(0, ControlType.kDutyCycle);
    targetRPM = 0.0;
  }

  @Override
  public void simulationPeriodic() {
    motor1Sim.setAppliedOutput(closedLoopController.getSetpoint());
    motor1Sim.iterate(closedLoopController.getSetpoint(), 12, 0.02);
  }
  
  /* readyToShoot - helper function to determine if the shooter is up to speed and ready to shoot.
   * @param rpmTolerance (double) the tolerance in rpm for determining if the shooter is up to speed.
   * @return (boolean) true if the shooter is up to speed, false otherwise.
   */
  public boolean readyToShoot(double rpmTolerance) {
    return Math.abs(currentRPM - targetRPM) < rpmTolerance;
  }

  public double getTargetRPM() {
    return targetRPM;
  }

}