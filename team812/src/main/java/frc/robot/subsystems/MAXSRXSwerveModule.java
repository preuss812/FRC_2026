// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.Math;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix6.StatusSignal;

import com.ctre.phoenix6.hardware.CANcoder;

import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.CANConstants;

public class MAXSRXSwerveModule {
  private final SparkMax m_drivingSparkMax;
  private final SparkMax m_turningSparkMax;
  private final SparkMaxConfig m_drivingConfig;
  private final SparkMaxConfig m_turningConfig;


  private final RelativeEncoder m_drivingEncoder;
  private final CANcoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingPIDController;
  private PIDController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
  private boolean debug = true;

  /**
   * Constructs a MAXSRXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSRXSwerveModule(int drivingCANId, int turningCANId, int turningEncoderCANId, double chassisAngularOffset) {

    m_drivingSparkMax = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);
    m_drivingConfig = new SparkMaxConfig(); // New for 2025
    m_turningConfig = new SparkMaxConfig(); // New for 2025

    // get the encoder objects for easier access later.
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = new CANcoder(turningEncoderCANId);

    // Get the PID controllers for the driving and turning motors.
    m_drivingPIDController = m_drivingSparkMax.getClosedLoopController();
    m_turningPIDController = new PIDController(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD);
    
    // TODO remove the redundant code below
    // Set the PID gains for the turning motor. Algae these are example gains, and you
    // may need to tune them for your own robot!
    m_turningPIDController.setP(ModuleConstants.kTurningP);
    m_turningPIDController.setI(ModuleConstants.kTurningI);
    m_turningPIDController.setD(ModuleConstants.kTurningD);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    // Match the PID's continuous range to [-pi, pi] to align with normalized angle below
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    // Reduce dithering near setpoint
    m_turningPIDController.setTolerance(Units.degreesToRadians(2.0));

    // Configure the driving Spark Max
    m_drivingConfig
      .inverted(false)
      .idleMode(ModuleConstants.kDrivingMotorIdleMode)
      //.closedLoopRampRate(10.0) # did nothing
      //.openLoopRampRate(10.0)   # did nothing
      ;

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    m_drivingConfig.encoder
      .positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor)
      .velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

    m_drivingConfig.smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);

    // Set the PID related parameters for the driving motor.
    m_drivingConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pidf(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD, ModuleConstants.kDrivingFF)
    .outputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput);
  
    m_drivingSparkMax.configure(m_drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    // Configure the turning Spark Max
    m_turningConfig
      .inverted(false)
      .idleMode(ModuleConstants.kDrivingMotorIdleMode); // Should this be brake or coast?
      m_turningConfig.smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    m_turningSparkMax.configure(m_turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    m_turningEncoder.setPosition(m_turningEncoder.getAbsolutePosition().getValue()); // An attempt to make the position == the absolute position
    m_desiredState.angle = new Rotation2d(this.CANCoderPositionRadians()); // Set desired angle to current angle so it wont move.

    m_drivingEncoder.setPosition(0);
    m_turningConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
      .pid(ModuleConstants.kTurningP,ModuleConstants.kTurningI,ModuleConstants.kTurningD)
      ;
  }
  /**
   * Helper function that converts CANCoder output into an angle in radians
   * in the range of 0 to 2*pi.
   * The CANCoder returns values in units of rotation, ie 1.0 = 1 full roration counterclockwise from the 0.0 position.
   * @return The current turning direction in radians
   */
  public double CANCoderPositionRadians() {
    // Read absolute position as rotations, convert to radians, then normalize to [-pi, pi)
    StatusSignal<Angle> angleStatus = m_turningEncoder.getAbsolutePosition();
    double rotations = angleStatus.getValueAsDouble();
    double angleRadians = rotations * 2.0 * Math.PI;
    double normalized = MathUtil.angleModulus(angleRadians);

    if (debug) SmartDashboard.putNumber("Absolute position"+m_turningEncoder.getDeviceID(), normalized);
    if (debug && (m_turningEncoder.getDeviceID() == CANConstants.kSwerveLeftFrontCANCoder)) {
      SmartDashboard.putNumber("lf_angle_radians", normalized);
      SmartDashboard.putNumber("lf_angle_degrees", normalized/(2*Math.PI)*360.0);
    }
    return normalized;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    double turningEncoderAngleRadians;
    turningEncoderAngleRadians = this.CANCoderPositionRadians();
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(turningEncoderAngleRadians - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    double turningEncoderAngleRadians;
    turningEncoderAngleRadians = this.CANCoderPositionRadians();
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(turningEncoderAngleRadians - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    double currentTurningAngleInRadians = this.CANCoderPositionRadians();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

   // TODO: Rework the optimize function to use the non-deprecated version
   // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(currentTurningAngleInRadians));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningPIDController.setSetpoint(optimizedDesiredState.angle.getRadians());
    
    double pidOutput = m_turningPIDController.calculate(currentTurningAngleInRadians);
    // Avoid dithering at the setpoint from encoder noise
    if (m_turningPIDController.atSetpoint()) {
      m_turningSparkMax.set(0.0);
    } else {
      m_turningSparkMax.set(pidOutput);
    }

    if (debug && (m_turningEncoder.getDeviceID() == CANConstants.kSwerveLeftFrontCANCoder)) {
      SmartDashboard.putNumber("optimizedTurn", optimizedDesiredState.angle.getRadians());
      SmartDashboard.putNumber("optimizedTurnError", optimizedDesiredState.angle.getRadians()-currentTurningAngleInRadians);
      SmartDashboard.putNumber("pidOutput", pidOutput);
    }

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  public void zeroOrientation() {
    
  }
  public void quiesce() {
    setDesiredState(m_desiredState); // Tell the module to want to be where it already is.
  }
}
