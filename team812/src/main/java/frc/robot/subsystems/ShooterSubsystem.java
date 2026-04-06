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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.ShooterMode;
import frc.robot.RobotContainer;
import frc.utils.ExponentialSmoother;

public class ShooterSubsystem extends SubsystemBase {


  // Shooter settings
  //private final double maxRPM = 5320; // For a CIM motor. 
  private double shooterTargetRPM = 0.0;  // desired shooter wheel speed
  private double shooterRPM = 0.0; // the current rpm of the shooter.
  private double feederTargetRPM = 0.0; // the current rpm of the feeder.
  private double feederRPM = 0.0; // the current rpm of the feeder.

  private double shooterCorrection = 0.0; // A correction factor in RPM to adjust the distance used for the RPM calculation.
  // native units are ticks per 100ms.
  // With external through bore encoder this calculation would be more typical:
  //  RPMToNativeUnits = 1 rpm * (4096 ticks/rpm) * (1 minute/60 seconds) * (0.1 second/100ms)
  private final double RPMToNativeUnits = 1.0;
  private final double nativeUnitsToRPM = 1.0/RPMToNativeUnits; // Typical CTRE Mag Encoder CPR
  
  
  private final SparkFlex feederMotor;
  private final SparkFlexSim feederMotorSim;
  private final DCMotor m_feederDCMotorForSim = DCMotor.getNeoVortex(1);
  
  private final SparkFlex followerMotor;
  private final SparkFlexSim followerSim;
  private final DCMotor m_followerDCMotorForSim = DCMotor.getNeoVortex(1);
  
  private final SparkFlex shooterMotor;
  private final SparkFlexSim shooterMotorSim;
  private final DCMotor m_shooterDCMotorForSim = DCMotor.getNeoVortex(1);
  private final SparkFlexConfig shooterMotorConfig;

    // Only used if there are dual motors on the shooter.
  

  private ShooterConstants.ShooterMode m_shooterMode = ShooterConstants.ShooterMode.IDLE;
  //private ShooterConstants.ShooterMode m_savedShooterMode = ShooterConstants.ShooterMode.IDLE;
  private final ExponentialSmoother smoothedRange = new ExponentialSmoother(1.0); // Not smoothing for now.
  private final double pwl[] = { // array for rpm vs feet from hub
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
  private final SparkFlexConfig followerMotorConfig;
  private final SparkFlexConfig feederMotorConfig;

  private final SparkClosedLoopController shooterClosedLoopController;
  private final SparkClosedLoopController followerClosedLoopController;
  private final SparkClosedLoopController feederClosedLoopController;

  private final RelativeEncoder shooterEncoder;
  private final RelativeEncoder feederEncoder;
  
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(int shooterCANId, int followerCANId,int feederCANId) {

  /*
     * Initialize the SPARK MAXes and get encoders and closed loop controllers
     * objects for later use.
     */
    shooterMotor = new SparkFlex(shooterCANId, MotorType.kBrushless);
    shooterMotorSim = new SparkFlexSim(shooterMotor, m_shooterDCMotorForSim);
    shooterClosedLoopController = shooterMotor.getClosedLoopController();
    shooterEncoder = shooterMotor.getEncoder();
    shooterMotorConfig = new SparkFlexConfig();
    shooterMotorConfig.closedLoopRampRate(1.0);
    shooterMotorConfig.idleMode(IdleMode.kCoast);
    shooterMotorConfig.inverted(ShooterConstants.inverted);
    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    shooterMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for velocity control in slot 1
        .p(ShooterConstants.kP)
        .i(ShooterConstants.kI)
        .d(ShooterConstants.kD)
        .outputRange(ShooterConstants.minOutputPercent, ShooterConstants.maxOutputPercent)
        .feedForward
          // kV is now in Volts, so we multiply by the nominal voltage (12V)
          .kV(ShooterConstants.kV)
          .kS(ShooterConstants.kS);
    shooterMotor.configure(shooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);


    if (followerCANId != -1) {
      followerMotor = new SparkFlex(followerCANId, MotorType.kBrushless);
      followerSim = new SparkFlexSim(followerMotor, m_followerDCMotorForSim);
      followerClosedLoopController = followerMotor.getClosedLoopController();
      followerMotorConfig = new SparkFlexConfig();
      followerMotorConfig.closedLoopRampRate(1.0);
      followerMotorConfig.idleMode(IdleMode.kCoast);
      followerMotorConfig.inverted(ShooterConstants.followerInverted);
      /*
      * Configure the closed loop controller. We want to make sure we set the
      * feedback sensor as the primary encoder.%
      */
      followerMotorConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for velocity control in slot 1
          .p(ShooterConstants.kP)
          .i(ShooterConstants.kI)
          .d(ShooterConstants.kD)
          .outputRange(ShooterConstants.minOutputPercent, ShooterConstants.maxOutputPercent)
          .feedForward
            // kV is now in Volts, so we multiply by the nominal voltage (12V)
            .kV(ShooterConstants.kV);
      followerMotorConfig.follow(shooterMotor, true);
      followerMotor.configure(followerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    } else {
      // No follower motor, so set all the follower variables to null.
      followerMotor = null;
      followerSim = null;
      followerClosedLoopController = null;
      followerMotorConfig = null;
    }

    feederMotor = new SparkFlex(feederCANId, MotorType.kBrushless);
    feederMotorSim = new SparkFlexSim(feederMotor, m_feederDCMotorForSim);
    feederClosedLoopController = feederMotor.getClosedLoopController();
    feederEncoder = feederMotor.getEncoder();
    feederMotorConfig = new SparkFlexConfig();
    feederMotorConfig.closedLoopRampRate(1.0);
    feederMotorConfig.inverted(FeederConstants.inverted);
    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    feederMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for velocity control in slot 1
        .p(FeederConstants.kP)
        .i(FeederConstants.kI)
        .d(FeederConstants.kD)
        .outputRange(FeederConstants.minOutputPercent, FeederConstants.maxOutputPercent)
        .feedForward
          // kV is now in Volts, so we multiply by the nominal voltage (12V)
          .kV(FeederConstants.kV);
    feederMotor.configure(feederMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    shooterRPM = 0.0;
    feederRPM = 0.0;
    SmartDashboard.putNumber("Shooter Correction", shooterCorrection);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //double rpm = SmartDashboard.getNumber("Shooter RPM Target", 0.0);
   //setRPM(rpm);
    // closed-loop velocity control
    // telemetry
    shooterRPM = shooterEncoder.getVelocity();
    SmartDashboard.putNumber("Shooter RPM", shooterRPM);
    feederRPM = feederEncoder.getVelocity();
    SmartDashboard.putNumber("Feeder RPM", feederRPM);
    
    //Distance from the center of the robot (Adjust later)
    Pose2d robotPose = RobotContainer.m_poseEstimatorSubsystem.getCurrentPose();
    Translation2d hubPos = AllianceConfigurationSubsystem.getHubCenter();
    double shooterOffset = 0;
    double distance = robotPose.getTranslation().getDistance(hubPos) - shooterOffset;
    SmartDashboard.putString("Distance to Hub", String.format("%1dft %1din", (int)Units.metersToInches(distance)/12, (int)Units.metersToInches(distance)%12));
    
    // This next section seemed like a good idea but there are unresolved bugs: 
    // Manage shooter mode based on the clock.  Active when we can score, otherwise inactive.
    // If we are unjamming or fixed speed mode, leave it as it is.
    /*
    if (AllianceConfigurationSubsystem.hubActiveSoon()) {
      if (m_shooterMode == ShooterConstants.ShooterMode.IDLE) 
        m_shooterMode = ShooterConstants.ShooterMode.AUTO_RANGING;
    } else if (!AllianceConfigurationSubsystem.hubActive()) {
      if (m_shooterMode == ShooterConstants.ShooterMode.AUTO_RANGING) {
        m_shooterMode = ShooterConstants.ShooterMode.IDLE;
      }
    }*/
    SmartDashboard.putString("ShooterMode", m_shooterMode.name());

    switch(m_shooterMode) {
      case IDLE -> {
        stop();  // De-energize the motor.
        smoothedRange.reset(); // Forget the past smoothing.
      }
      case AUTO_RANGING -> {
        double correction = getShooterCorrection();
        double adjustedDistance = smoothedRange.addSample(distance);
        double RPM = distanceToRPMPWL(MathUtil.clamp(adjustedDistance, 1.0, 6.0)) + correction; // The RPM is not fit beyond this range.
        setRPM(RPM);
      }
      case FIXED_SPEED -> {
        setRPM(shooterTargetRPM);
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
    shooterClosedLoopController.setSetpoint(pOut, ControlType.kDutyCycle);
  }

  public double rpmToNativeUnits(double rpm) {
    return RPMToNativeUnits * rpm;
  }

  /**
   * setRPM - set the target rpm
   * @param rpm (double) The rpm target for the shooter
   */
  public void setRPM(double rpm) {
    shooterTargetRPM = rpm;
    feederTargetRPM = (rpm < 0) ? shooterTargetRPM : shooterTargetRPM*FeederConstants.shooterFactor;
    double targetVelocity = rpmToNativeUnits(shooterTargetRPM);
    // If rpm is negative, we are unjamming so run the feeder at the same speed as the shooter. Otherwise, run the feeder at a fraction of the shooter speed.
    double feederTargetVelocity = rpmToNativeUnits(feederTargetRPM);
    shooterClosedLoopController.setSetpoint(targetVelocity, ControlType.kVelocity);
    SmartDashboard.putNumber("Shooter RPM Target", shooterTargetRPM);
    feederClosedLoopController.setSetpoint(feederTargetVelocity, ControlType.kVelocity);
    SmartDashboard.putNumber("Feeder RPM Target", feederTargetRPM);
  }

  public void setFixedRPM(double rpm) {
    setShooterMode(ShooterConstants.ShooterMode.FIXED_SPEED);
    setRPM(rpm);
  }

  public double getShooterRPM() {
    return shooterRPM * nativeUnitsToRPM;
  }

  public double getFeederRPM() {
    return feederRPM * nativeUnitsToRPM;
  }

  public void stop() {
    shooterClosedLoopController.setSetpoint(0, ControlType.kDutyCycle);
    if (followerMotor != null) 
      followerClosedLoopController.setSetpoint(0, ControlType.kDutyCycle);
    feederClosedLoopController.setSetpoint(0, ControlType.kDutyCycle);
    m_shooterMode = ShooterConstants.ShooterMode.IDLE;
    shooterTargetRPM = 0.0;
    feederTargetRPM = 0.0;
    SmartDashboard.putNumber("Shooter RPM Target", shooterTargetRPM);
    SmartDashboard.putNumber("Feeder RPM Target", feederTargetRPM);
  }

  @Override
  public void simulationPeriodic() {
    shooterMotorSim.setAppliedOutput(shooterClosedLoopController.getSetpoint());
    shooterMotorSim.iterate(shooterClosedLoopController.getSetpoint(), 12, 0.02);
    feederMotorSim.setAppliedOutput(feederClosedLoopController.getSetpoint());
    feederMotorSim.iterate(feederClosedLoopController.getSetpoint(), 12, 0.02);
    if (followerMotor != null) {
      followerSim.setAppliedOutput(followerClosedLoopController.getSetpoint());
      followerSim.iterate(followerClosedLoopController.getSetpoint(), 12, 0.02);
    }
  }
  
  public boolean shooterReadyToShoot() {
    return Math.abs(shooterRPM - shooterTargetRPM) < ShooterConstants.RPMTolerance;
  }
  
  public boolean feederReadyToShoot() {
    return Math.abs(feederRPM - feederTargetRPM) < FeederConstants.RPMTolerance;
  }

  public double shooterTargetRPM() {
    return shooterTargetRPM;
  }

  public double feederTargetRPM() {
    return feederTargetRPM;
  }

  public double shooterError() {
    return shooterRPM - shooterTargetRPM;
  } 

  public double feederError() {
    return feederRPM - feederTargetRPM;
  } 
  
  /* readyToShoot - helper function to determine if the shooter is up to speed and ready to shoot.
   * @param rpmTolerance (double) the tolerance in rpm for determining if the shooter is up to speed.
   * @return (boolean) true if the shooter is up to speed, false otherwise.
   */
  public boolean readyToShoot() {
    return (Math.abs(shooterRPM - shooterTargetRPM) < ShooterConstants.RPMTolerance)
        && (Math.abs(feederRPM - shooterTargetRPM*FeederConstants.shooterFactor) < FeederConstants.RPMTolerance);
  }

  public double getShooterTargetRPM() {
    return shooterTargetRPM;
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

  public double getShooterCorrection() {
    return shooterCorrection;
  }

  public void incrementShooterCorrection(double increment) {
    shooterCorrection += increment;
    SmartDashboard.putNumber("Shooter Correction", shooterCorrection);
  }

}