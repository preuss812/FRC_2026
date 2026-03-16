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
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {



    // Shooter settings
    //private final double maxRPM = 5320; // For a CIM motor. 
    private double targetRPM = 0.0;  // desired shooter wheel speed
    private double savedTargetRPM = 0.0;
    private double currentRPM; // the current rpm of the shooter.
    // native units are ticks per 100ms.
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
    private final int stuckMotorThreshold = 25; // 1/2 second
    private int stuckMotorCount = 0;
    private final int stuckMotorCoolDownDelay = 50*5; // 5 Seconds
    private int stuckMotorCoolDownCount = 0;
    private final double stuckMotorPercentOkay = 0.5; // Motor must spin at least 1/2 of what we asked for.

  public ShooterSubsystem(int shooterCANId) {
      /** Creates a new ShooterSubsystem. */

  /*
     * Initialize the SPARK MAX and get its encoder and closed loop controller
     * objects for later use.
     */
    motor1 = new SparkFlex(CANConstants.kShooterMotor1, MotorType.kBrushless);
    motor1Sim = new SparkFlexSim(motor1, m_dcMotor);

    //motor2 = new SparkFlex(CANConstants.kShooterMotor2, MotorType.kBrushless);
    closedLoopController = motor1.getClosedLoopController();
    encoder = motor1.getEncoder();

    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    motorConfig = new SparkFlexConfig();
    //motorFollowerConfig = new SparkFlexConfig();
    motorConfig.inverted(true);
    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    /*
    motorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);
    */
   /*
     * Configure the 2nd motor on the Shooter to follow, inverted, the primary motor
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
        .p(ShooterConstants.kP)
        .i(ShooterConstants.kI+0.000)
        .d(ShooterConstants.kD)
        //.iMaxAccum(50.0)
        //.iZone(100)
        .outputRange(ShooterConstants.minOutputPercent, ShooterConstants.maxOutputPercent)
        .feedForward
          // kV is now in Volts, so we multiply by the nominal voltage (12V)
          .kV(ShooterConstants.kV);

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
    SmartDashboard.setDefaultNumber("Shooter RPM Target", 0);
    currentRPM = 0;
    SmartDashboard.putBoolean("Shooter OK", true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //double rpm = SmartDashboard.getNumber("Shooter RPM Target", 0.0);
   //setRPM(rpm);
    // closed-loop velocity control
    currentRPM = encoder.getVelocity();
    // telemetry
    SmartDashboard.putNumber("Shooter RPM", currentRPM);
    //SmartDashboard.putNumber("Shooter Vout", motor1.getAppliedOutput());
    //SmartDashboard.putNumber("Throttle", RobotContainer.leftJoystick.getRawAxis(3));

    // = SmartDashboard.getNumber("Target Velocity", 0);
    //closedLoopController.setSetpoint(targetRPM, ControlType.kVelocity);
    // Check for a jammed motor.
    /*
    if (stuckMotorCoolDownCount > 0) {
      stop(); // Keep reapplying stop to be sure we are stopped.
      stuckMotorCoolDownCount--;
      if (stuckMotorCoolDownCount <= 0) {
        stuckMotorCoolDownCount = 0; // just to be sure.
        setRPM(savedTargetRPM);
        SmartDashboard.putBoolean("Shooter OK", true);
        stuckMotorCount = 0;
      }
    } else {
      if (Math.abs(currentRPM) < stuckMotorPercentOkay * Math.abs(targetRPM)) {
        stuckMotorCount++;
        if (stuckMotorCount >= stuckMotorThreshold) {
          savedTargetRPM = targetRPM;
          stop();
          SmartDashboard.putBoolean("Shooter OK", false);
          stuckMotorCoolDownCount = stuckMotorCoolDownDelay;
        }
      } else {
        stuckMotorCount = 0; // reset the counter we are not stuck.
      }
    }
      */
  }

  public void runMotor(double pOut){
    pOut = MathUtil.clamp(pOut, ShooterConstants.minOutputPercent, ShooterConstants.maxOutputPercent);
    closedLoopController.setSetpoint(pOut, ControlType.kDutyCycle);
  }

  public double rpmToNativeUnits(double rpm) {
    return RPMToNativeUnits * rpm;
  }

  /**
   * setRPM - set the target rpm
   * @param rpm (double) The rpm target for the shooter
   */
  public void setRPM(double rpm) {
    targetRPM = rpm;
    double targetVelocity = rpmToNativeUnits(rpm);
    closedLoopController.setSetpoint(targetVelocity, ControlType.kVelocity);
    SmartDashboard.putNumber("Shooter RPM Target", targetRPM);
    
  }

  public double getRPM() {
    return currentRPM * nativeUnitsToRPM;
  }

  public void stop() {
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