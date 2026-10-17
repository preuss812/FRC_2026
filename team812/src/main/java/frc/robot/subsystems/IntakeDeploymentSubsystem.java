// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeDeploymentConstants;

public class IntakeDeploymentSubsystem extends SubsystemBase {

    // IntakeDeployment settings
    private double targetRPM = 0.0;  // desired intakedeployment wheel speed
    private double targetPosition = 0.0; // desired intakedeployment position in rotations, where 0.0 is fully raised position.
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
    private double m_position = 0.0; // 0.0 is the fully raised position.
    private boolean m_holdDown = false; // true while the intake is deployed and should be clamped against the bottom hard stop.
    private boolean m_holdActive = false; // true while periodic() is regulating the holding current.
    private double m_holdDutyCycle = 0.0; // current holding duty cycle (negative is down).
    private final boolean debug = true;

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
    motorConfig.limitSwitch
        .forwardLimitSwitchTriggerBehavior(LimitSwitchConfig.Behavior.kKeepMovingMotor)
        .reverseLimitSwitchTriggerBehavior(LimitSwitchConfig.Behavior.kKeepMovingMotor);
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
        // Set PID values for velocity control in slot 0
        .p(IntakeDeploymentConstants.kP)
        .i(IntakeDeploymentConstants.kI)
        .d(IntakeDeploymentConstants.kD)
        .outputRange(IntakeDeploymentConstants.minOutputPercent, IntakeDeploymentConstants.maxOutputPercent)
        // Set PID values for position control in slot 1
        .p(0.1, ClosedLoopSlot.kSlot1)
        .i(0.0, ClosedLoopSlot.kSlot1)
        .d(0.0, ClosedLoopSlot.kSlot1)
        .outputRange(IntakeDeploymentConstants.minOutputPercent, IntakeDeploymentConstants.maxOutputPercent, ClosedLoopSlot.kSlot1)

        .feedForward
          // kV is now in Volts, so we multiply by the nominal voltage (12V)
          .kV(IntakeDeploymentConstants.kV);
      motorConfig.smartCurrentLimit(IntakeDeploymentConstants.currentLimit);

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
    SmartDashboard.putNumber("Intake Deployment Percent Output", motor1.getAppliedOutput());
    SmartDashboard.putNumber("Intake Deployment Motor Current", motor1.getOutputCurrent());
    //double rpm = SmartDashboard.getNumber("IntakeDeployment RPM Target", 0.0);
    //setRPM(rpm);
    // closed-loop velocity control
    currentRPM = encoder.getVelocity();
    // telemetry
    SmartDashboard.putNumber("IntakeDeployment RPM", currentRPM);
    m_position = encoder.getPosition();
    SmartDashboard.putNumber("Intake Position", m_position);

    double armAngle = IntakeDeploymentConstants.minRotation + (m_position - IntakeDeploymentConstants.minPosition) * IntakeDeploymentConstants.positionToRotationFactor;
    double feedForward = IntakeDeploymentConstants.maxFeedForwardPercent * Math.cos(armAngle);
    SmartDashboard.putNumber("Intake Angle", Units.radiansToDegrees(armAngle));
    SmartDashboard.putNumber("Intake Feed Forward Position", feedForward);
    SmartDashboard.putString("Intake Control Type", closedLoopController.getControlType().toString());


    // Set feedforward based on operating mode.
    if (closedLoopController.getControlType() == ControlType.kVelocity) {
      SmartDashboard.putNumber("IntakeDeployment Target RPM", targetRPM);
      closedLoopController.setSetpoint(targetRPM, ControlType.kVelocity); // Not using feedforward although it would possibly help.
    } else if (closedLoopController.getControlType() == ControlType.kPosition) {
      SmartDashboard.putNumber("IntakeDeployment Target Position", closedLoopController.getSetpoint());
      closedLoopController.setSetpoint(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot1, feedForward);
    }
    // Regulate the holding duty cycle so the holding current stays near the target
    // and cannot creep up over time.
    if (m_holdActive) {
      double currentError = IntakeDeploymentConstants.kIntakeDeploymentHoldDownCurrent - Math.abs(motor1.getOutputCurrent());
      m_holdDutyCycle = MathUtil.clamp(
        m_holdDutyCycle - IntakeDeploymentConstants.kIntakeDeploymentHoldDownGain * currentError,
        IntakeDeploymentConstants.kIntakeDeploymentHoldDownDutyCycle,
        IntakeDeploymentConstants.kIntakeDeploymentMinHoldDownDutyCycle);
      closedLoopController.setSetpoint(m_holdDutyCycle, ControlType.kDutyCycle);
    }
    SmartDashboard.putNumber("Intake Hold Duty Cycle", m_holdActive ? m_holdDutyCycle : 0.0);
    // = SmartDashboard.getNumber("Target Velocity", 0);
    m_atFwdLimit = m_fwdLimitSwitch.isPressed();
    m_atRevLimit = m_revLimitSwitch.isPressed();
    SmartDashboard.putBoolean("IntakeFwdLimit", m_atFwdLimit);
    SmartDashboard.putBoolean("IntakeRevLimit", m_atRevLimit); 
    SmartDashboard.putBoolean("IntakeUp", m_atFwdLimit);
    SmartDashboard.putBoolean("IntakeDown", m_atRevLimit);
    SmartDashboard.putBoolean("Intake Hold Down", m_holdDown);
    SmartDashboard.putString("Intake Deployment Command", getCurrentCommand() == null ? "none" : getCurrentCommand().getName());
  }

  public boolean isRevLimitSwitchPressed() {
    return m_atRevLimit;
  }

  public void resetEncoder() {
    encoder.setPosition(0.0);
  }

  public void resetEncoder(double position) {
    encoder.setPosition(position);
  }

  public double getOutputCurrent() {
    return motor1.getOutputCurrent();
  }

  public double getPosition() {
    return m_position;
  }

  public void runMotor(double pOut){
    m_holdActive = false;
    targetRPM = 0.0;
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
    m_holdActive = false;
    targetRPM = rpm;
    double targetVelocity = rpmToNativeUnits(rpm);
    closedLoopController.setSetpoint(targetVelocity, ControlType.kVelocity);
    SmartDashboard.putNumber("IntakeDeployment Target RPM", targetRPM);
  }

  public double getRPM() {
    return currentRPM * nativeUnitsToRPM;
  }

  public void setPosition(double targetPosition) {
    m_holdActive = false;
    m_position = encoder.getPosition();
    SmartDashboard.putNumber("Intake Position", m_position);
    double armAngle = IntakeDeploymentConstants.minRotation + (m_position - IntakeDeploymentConstants.minPosition) * IntakeDeploymentConstants.positionToRotationFactor;
    double feedForward = IntakeDeploymentConstants.maxFeedForwardPercent * Math.cos(armAngle);
    this.targetPosition = MathUtil.clamp(targetPosition, IntakeDeploymentConstants.minPosition, IntakeDeploymentConstants.maxPosition);
    closedLoopController.setSetpoint(targetPosition, ControlType.kPosition);//, ClosedLoopSlot.kSlot1, feedForward);
    SmartDashboard.putNumber("IntakeDeployment Target Position", this.targetPosition);
  }

  /**
   * stop - stop active motion. If the intake is deployed (hold down mode), keep applying
   * the gentle hold-down torque instead of cutting power so the arm stays pinned against
   * the bottom hard stop when a command using the intake ends or is interrupted.
   */
  public void stop() {
    if (m_holdDown) {
      holdDown();
    } else {
      m_holdActive = false;
      closedLoopController.setSetpoint(0, ControlType.kDutyCycle);
    }
    targetRPM = 0.0;
  }

  /**
   * holdDown - clamp the arm against the bottom hard stop with a gentle, current
   * regulated holding torque. periodic() keeps adjusting the duty cycle to hold the
   * target current until another motion method is called. Safe to call every loop.
   */
  public void holdDown() {
    if (!m_holdActive) {
      m_holdActive = true;
      m_holdDutyCycle = IntakeDeploymentConstants.kIntakeDeploymentHoldDownDutyCycle;
      closedLoopController.setSetpoint(m_holdDutyCycle, ControlType.kDutyCycle);
    }
    targetRPM = 0.0;
  }

  /**
   * setHoldDown - set whether the intake is deployed and should be held down.
   * @param holdDown (boolean) true when lowering the intake, false when raising it.
   */
  public void setHoldDown(boolean holdDown) {
    m_holdDown = holdDown;
  }

  public boolean isHoldDown() {
    return m_holdDown;
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