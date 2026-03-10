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
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeDeploymentConstants;

public class IntakeDeploymentSubsystem extends SubsystemBase {

    // IntakeDeployment settings
    private double targetRPM = 0.0;  // desired intakedeployment wheel speed
    private double currentRPM; // the current rpm of the intakedeployment.
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
    private final SparkLimitSwitch m_fwdLimitSwitch;
    private final SparkLimitSwitch m_revLimitSwitch;
    private boolean m_atFwdLimit = true; // dont allow motion until after periodic sets this.
    private boolean m_atRevLimit = true; // dont allow motion until after periodic sets this.

  public IntakeDeploymentSubsystem(int intakedeploymentCANId) {
      /** Creates a new IntakeDeploymentSubsystem. */

    /*
     * Initialize the SPARK MAX and get its encoder and closed loop controller
     * objects for later use.
     */
    motor1 = new SparkFlex(intakedeploymentCANId, MotorType.kBrushless);
    motor1Sim = new SparkFlexSim(motor1, m_dcMotor);
    
    closedLoopController = motor1.getClosedLoopController();
    encoder = motor1.getEncoder();

    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    motorConfig = new SparkFlexConfig();
    //motorFollowerConfig = new SparkFlexConfig();
    motorConfig.inverted(false); // This makes negative RPM deploy the shooter
    motorConfig.idleMode(IdleMode.kBrake);
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
     * Configure the 2nd motor on the IntakeDeployment to follow, inverted, the primary motor
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
        .p(IntakeDeploymentConstants.kP)
        .i(IntakeDeploymentConstants.kI)
        .d(IntakeDeploymentConstants.kD)
        .outputRange(IntakeDeploymentConstants.minOutputPercent, IntakeDeploymentConstants.maxOutputPercent)
        .feedForward
          // kV is now in Volts, so we multiply by the nominal voltage (12V)
          .kV(IntakeDeploymentConstants.kV);

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
    m_fwdLimitSwitch = motor1.getForwardLimitSwitch();
    m_revLimitSwitch = motor1.getReverseLimitSwitch();

    // Initialize dashboard values
    SmartDashboard.setDefaultNumber("IntakeDeployment RPM Target", 0);
    currentRPM = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double rpm = SmartDashboard.getNumber("IntakeDeployment RPM Target", 0.0);
    setRPM(rpm);
    // closed-loop velocity control
    currentRPM = encoder.getVelocity();
    // telemetry
    SmartDashboard.putNumber("IntakeDeployment RPM", currentRPM);

    // = SmartDashboard.getNumber("Target Velocity", 0);
    closedLoopController.setSetpoint(targetRPM, ControlType.kVelocity);
    m_atFwdLimit = m_fwdLimitSwitch.isPressed();
    m_atRevLimit = m_revLimitSwitch.isPressed();
    SmartDashboard.putBoolean("IntakeFwdLimit", m_atFwdLimit);
    SmartDashboard.putBoolean("IntakeRevLimit", m_atRevLimit); 
    SmartDashboard.putBoolean("IntakeUp", m_atFwdLimit);
    SmartDashboard.putBoolean("IntakeDown", m_atRevLimit);
  }

  public void runMotor(double pOut){
    pOut = MathUtil.clamp(pOut, IntakeDeploymentConstants.minOutputPercent, IntakeDeploymentConstants.maxOutputPercent);
    closedLoopController.setSetpoint(pOut, ControlType.kDutyCycle);
  }

  public double rpmToNativeUnits(double rpm) {
    return RPMToNativeUnits * rpm;
  }

  /**
   * setRPM - set the target rpm
   * @param rpm (double) The rpm target for the intakedeployment
   */
  public void setRPM(double rpm) {
    targetRPM = rpm;
    double targetVelocity = rpmToNativeUnits(rpm);
    closedLoopController.setSetpoint(targetVelocity, ControlType.kVelocity);
    SmartDashboard.putNumber("IntakeDeployment Target RPM", targetRPM);
  }

  public double getRPM() {
    return currentRPM * nativeUnitsToRPM;
  }

  public void stop() {
    closedLoopController.setSetpoint(0, ControlType.kDutyCycle);
  }

  @Override
  public void simulationPeriodic() {
    motor1Sim.setAppliedOutput(closedLoopController.getSetpoint());
    motor1Sim.iterate(closedLoopController.getSetpoint(), 12, 0.02);
  }
  
  /*
   * fullyRaised - return true if the intake is fully raised (i.e. at the forward limit).
   * @return (boolean) true if the intake is fully raised, false otherwise.
   */
  public boolean fullyRaised() {
    return m_atFwdLimit;
  }

  /*
   * fullyLowered - return true if the intake is fully lowered (i.e. at the reverse limit).
   * @return (boolean) true if the intake is fully lowered, false otherwise.
   */
  public boolean fullyLowered() {
    return m_atRevLimit;
  }

}