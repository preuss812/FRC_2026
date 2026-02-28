// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Robot;

public class ShooterSubsystem extends SubsystemBase {



    // Shooter settings
    //private final double maxRPM = 5320; // For a CIM motor. 
    private final double defaultRPM = 3000;
    private final double defaultRPMFeedForward = 1.0; // TODO: need a real number.
    private double targetRPM = defaultRPM;  // desired shooter wheel speed
    private double currentRPM; // the current rpm of the shooter.
    // native units are ticks per 100ms.
    // the revrobotics through bore encoder I think has 4096 ticks per revolution.
    // Therefore rpm to ticks is 1 rpm * (4096 ticks/rpm) * (1 minute/60 seconds) * (0.1 second/100ms) = rpm to native
    private final double RPMToNativeUnits = 1.0 * (4096.0) * (1.0/60.0) * (0.1);
    private final double nativeUnitsToRPM = 1.0/RPMToNativeUnits; // Typical CTRE Mag Encoder CPR

    private final int pidIdx = 0;
    private final int slotIdx = 0;
    private final int timeoutMs = 10;
    private final SparkFlex motor1;
    //private final SparkFlex motor2;
    private final SparkFlexConfig motorConfig;
    //private final SparkFlexConfig motorFollowerConfig;
    private final SparkClosedLoopController closedLoopController;
    private final RelativeEncoder encoder;
    

  public ShooterSubsystem(int shooterCANId) {
      /** Creates a new ShooterSubsystem. */

  /*
     * Initialize the SPARK MAX and get its encoder and closed loop controller
     * objects for later use.
     */
    motor1 = new SparkFlex(CANConstants.kShooterMotor1, MotorType.kBrushless);
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
    motorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);
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
        .p(0.0002)
        //.i(0)
        //.d(0)
        .outputRange(-1, 1)
        .feedForward
          // kV is now in Volts, so we multiply by the nominal voltage (12V)
          .kV(1.0 / 6784);

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
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double rpm = SmartDashboard.getNumber("Shooter RPM Target", 0.0);
    setRPM(rpm);
    // closed-loop velocity control
    if (Robot.isReal()) currentRPM = encoder.getVelocity();
    // telemetry
    SmartDashboard.putNumber("Shooter RPM", currentRPM);

    // = SmartDashboard.getNumber("Target Velocity", 0);
    closedLoopController.setSetpoint(targetRPM, ControlType.kVelocity);
  }

  public void runMotor(double pOut){
    pOut = MathUtil.clamp(pOut, -1, 1);
    closedLoopController.setSetpoint(pOut, ControlType.kDutyCycle);
  }

  public double rpmToNativeUnits(double rpm) {
    return 1.0 * rpm;
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
    return currentRPM;
  }
  

  public double rpmToFeedForward(double rpm) { 
     //TODO: find a good value/equation for the conversion.
    double feedForward = rpm / defaultRPM * defaultRPMFeedForward;
    return feedForward;
  }

  public void stop() {
    closedLoopController.setSetpoint(0, ControlType.kDutyCycle);
  }

  @Override
  public void simulationPeriodic() {
    currentRPM = currentRPM + (targetRPM - currentRPM) * 0.1; // simple simulation of the shooter speed changing over time.
  }
  
}