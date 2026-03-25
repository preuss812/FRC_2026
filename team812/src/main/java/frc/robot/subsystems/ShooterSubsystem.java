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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer;
import frc.utils.ExponentialSmoother;

public class ShooterSubsystem extends SubsystemBase {


  // Shooter settings
  //private final double maxRPM = 5320; // For a CIM motor. 
  private double targetRPM = 0.0;  // desired shooter wheel speed
  private double currentRPM; // the current rpm of the shooter.
  // native units are ticks per 100ms.
  // With external through bore encoder this calculation would be more typical:
  //  RPMToNativeUnits = 1 rpm * (4096 ticks/rpm) * (1 minute/60 seconds) * (0.1 second/100ms)
  private final double RPMToNativeUnits = 1.0;
  private final double nativeUnitsToRPM = 1.0/RPMToNativeUnits; // Typical CTRE Mag Encoder CPR
  private final SparkFlex motor1;
  private final SparkFlexSim motor1Sim;
  private final DCMotor m_dcMotor = DCMotor.getNEO(1);
  private ShooterConstants.ShooterMode m_shooterMode = ShooterConstants.ShooterMode.IDLE;
  private ShooterConstants.ShooterMode m_savedShooterMode = ShooterConstants.ShooterMode.IDLE;
  private final ExponentialSmoother smoothedRange = new ExponentialSmoother(1.0); // Not smoothing for now.
  private final double pwl[] = { // array or rpm vs feet from hub
    2500.0, // 0
    2500.0, // 1
    2500.0, // 2
    2500.0, // 3
    2500.0, // 4
    2500.0, // 5
    2500.0, // 6
    2550.0, // 7
    2600.0, // 8
    2695.0, // 9
    2790.0, // 10
    2875.0, // 11
    2970.0, // 12
    3090.0, // 13
    3200.0, // 14
    3300.0, // 15
    3400.0, // 16
    3500.0, // 17
    3600.0, // 18
    3700.0, // 19
    3800.0, // 20
  };
  //private final SparkFlex motor2;
  private final SparkFlexConfig motorConfig;
  //private final SparkFlexConfig motorFollowerConfig;
  private final SparkClosedLoopController closedLoopController;
  private final RelativeEncoder encoder;
  
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(int shooterCANId) {

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
    motorConfig.closedLoopRampRate(1.0);
    //motorFollowerConfig = new SparkFlexConfig();
    motorConfig.inverted(true);
    //motorConfig.smartCurrentLimit(ShooterConstants.currentLimit);
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
        .i(ShooterConstants.kI)
        .d(ShooterConstants.kD)
        //.iMaxAccum(50.0)
        //.iZone(50)
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
    //SmartDashboard.putBoolean("Shooter OK", true);
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
    /*
      // if the joystick is being used, save the current mode and switch to unjamming mode
      double joystickInput = RobotContainer.rightJoystick.getY();
      if (Math.abs(joystickInput) > 0.3) {
        if (m_shooterMode != ShooterConstants.ShooterMode.UNJAMMING) {
          m_savedShooterMode = m_shooterMode;
        }
        // If we are in unjamming mode and the joystick is not used, switch back to the saved mode.
      } else if (m_shooterMode == ShooterConstants.ShooterMode.UNJAMMING) {
        m_shooterMode = m_savedShooterMode;
      }
    */
    //Distance from the center of the robot (Adjust later)
    Pose2d robotPose = RobotContainer.m_poseEstimatorSubsystem.getCurrentPose();
    Translation2d hubPos = AllianceConfigurationSubsystem.getHubCenter();
    double shooterOffset = 0;
    double distance = robotPose.getTranslation().getDistance(hubPos) - shooterOffset;
    SmartDashboard.putString("Distance to Hub", String.format("%1dft %1din", (int)Units.metersToInches(distance)/12, (int)Units.metersToInches(distance)%12));
    
    // Manage shooter mode based on the clock.  Active when we can score, otherwise inactive.
    // If we are unjamming or fixed speed mode, leave it as it is.
    if (AllianceConfigurationSubsystem.hubActiveSoon()) {
      if (m_shooterMode == ShooterConstants.ShooterMode.IDLE) 
        m_shooterMode = ShooterConstants.ShooterMode.AUTO_RANGING;
    } else if (!AllianceConfigurationSubsystem.hubActive()) {
      if (m_shooterMode == ShooterConstants.ShooterMode.AUTO_RANGING) {
        m_shooterMode = ShooterConstants.ShooterMode.IDLE;
      }
    }

    switch(m_shooterMode) {
      case IDLE -> {
        stop();  // De-energize the motor.
        RobotContainer.m_FeederSubsystem.stop();
        smoothedRange.reset(); // Forget the past smoothing.
      }
      case AUTO_RANGING -> {
        double correction = RobotContainer.getShooterCorrection();
        double adjustedDistance = smoothedRange.addSample(distance+correction);
        double RPM = distanceToRPMPWL(MathUtil.clamp(adjustedDistance, 1.0, 6.0)); // The RPM is not fit beyond this range.
        setRPM(RPM);
        RobotContainer.m_FeederSubsystem.setRPM(RPM);
      }
      case FIXED_SPEED -> {
        setRPM(targetRPM);
      }
      case UNJAMMING -> {
        /*
        runMotor(joystickInput);
        RobotContainer.m_FeederSubsystem.runMotor(joystickInput);
        */
        setRPM(-6000);
        smoothedRange.reset(); // Forget the past smoothing.
      }
    }
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

  /**
   * distanceToRPMPWL - compute ideal motor speed given distance to the target.
   * @param x (double) the distance to the target in meters.
   * @return (double) the ideal RPM for shooting at that distance.
   */
  public double distanceToRPMPWL(double x) {
    double feet = Units.metersToFeet(x);
    int wholeFeet = (int)Math.floor(Units.metersToFeet(Math.abs(x)));
    double remainder = feet - wholeFeet;
    if (wholeFeet > pwl.length - 2)
      wholeFeet = pwl.length - 2;
    return pwl[wholeFeet] + (pwl[wholeFeet+1] - pwl[wholeFeet]) * remainder;
  }

  public void setShooterMode(ShooterConstants.ShooterMode shooterMode) {
    m_shooterMode = shooterMode;
  }
  
  public ShooterConstants.ShooterMode getShooterMode() {
    return m_shooterMode;
  }
}