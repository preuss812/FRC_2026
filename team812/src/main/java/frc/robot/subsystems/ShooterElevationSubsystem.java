// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AnalogIOConstants;
import frc.robot.Constants.PidConstants;
import frc.robot.Constants.ShooterElevationConstants;
import frc.robot.Utilities;
import frc.utils.PreussMotor;

public class ShooterElevationSubsystem extends SubsystemBase {
  public final PreussMotor m_shooterElevation = new PreussMotor(Constants.shooterElevationMotor);
  private double m_analogPosition;
  private double m_targetPosition;
  private double m_currentPosition;
  private boolean m_rotateStopped = true;
  private boolean m_capturedLimitPosition = true; // Due to absolute encoder, we are always homed.
  private AnalogInput m_analogInput = new AnalogInput(AnalogIOConstants.kShooterElevationEncoder);
  private PIDController m_pidController = new PIDController(PidConstants.kShooterElevation_kP, PidConstants.kShooterElevation_kI, PidConstants.kShooterElevation_kD);
  private boolean debug = true; // TODO: Set to false once the shooter is debugged.
  private final double INCREMENT_SIZE = 0.5; // 0.5*50cycles/sec = 25 degrees per second when joystick maxed out. TODO tune this
  private double m_percentOutput = 0.0;
  private boolean m_homing = false;
  private final double m_timestep = 0.020; // for simulation

  /** Creates a new ArmSubsystem. */
  public ShooterElevationSubsystem() { 
    stop(); // Make sure the motor is not moving
    readCurrentPosition();
    m_targetPosition = m_currentPosition;  // initially hold the starting shooter elevation position.
  }


  // This function is used as the default command to run for shooter elevation control
  // which is typically done by attaching to a joystick.
  // If the shooter is working well, this would be unnecessary.
  // The input is presumed to be an analog input that ranges from -1 to +1.
  public void rotate(double throttle) {
    //double absolutePosition = getPosition(); // Should get the goal, not the position. // Dont need it.
    // if the joystick is nearly centered, ignore it
    // This has the effect of stopping the shooter elevation rotation if the joystick is not being used to control the shooter elevation.
    // Be aware that if another command ends before it gets the shooter elevation to the desired position,
    // this function will stop the shooter elevation motiion and it will not continue rotating to the other commands target.
    if (Math.abs(throttle) < 0.1) {  // Also move to constants.java
      if (!m_rotateStopped) {
        setM_targetPosition(getM_currentPosition());
        m_rotateStopped = true;
      }
    } else {
      m_rotateStopped = false;
      double newPosition = m_targetPosition + throttle * INCREMENT_SIZE;
      newPosition = MathUtil.clamp(newPosition, ShooterElevationConstants.kShooterElevationMinPosition, ShooterElevationConstants.kShooterElevationMaxPosition);
      setM_targetPosition(newPosition);
    }
    m_shooterElevation.configSelectedFeedbackSensor(FeedbackDevice.Analog, ShooterElevationConstants.kPidIdx, ShooterElevationConstants.kTimeoutMs);
    
  };

  /**
   * stop - stop the motor.
   */
  public void stop() {
    m_percentOutput = 0.0;
    m_shooterElevation.set(ControlMode.PercentOutput, m_percentOutput);
  }

  /**
   * runMotor - run the motor at the specified output percentage.
   * @param speed - the percent motor output to use ranging from -1.0 to 1.0.
   */
  public void runMotor(double speed) {
    m_percentOutput = MathUtil.clamp(speed, ShooterElevationConstants.kShooterElevationPeakOutputReverse, ShooterElevationConstants.kShooterElevationPeakOutputForward);
    m_shooterElevation.set(ControlMode.PercentOutput, m_percentOutput);
  }

  /**
   * home
   * @param position
   * @return
   */
  public void home() {
    if (isHomed()) unsetHomed();
    m_homing = true;
    runMotor(ShooterElevationConstants.kShooterElevationHomeSpeed);
  }
  /**
   * setTargetPosition - Set the shooter elevation target position after checking that it is safe to do so.
   * @param - the target angle for the shooter elevation in degrees
   * @return - the current angle of the shooter elevation.
   */
  public double setM_targetPosition(double position) {
    // If the shooter is not homed, do not allow the target position to be set.
    if (isHomed()) {
      double clampedPosition = MathUtil.clamp(position, ShooterElevationConstants.kShooterElevationMinPosition, ShooterElevationConstants.kShooterElevationMaxPosition);
      m_targetPosition = clampedPosition;
    }
    return getM_currentPosition();
  }

  /**
   * incrementTargetPosition - add to the current target position
   * @param - increment (degrees) the amount to add to the target position.
   */
  public void incrementTargetPosition(double increment) {
    setM_targetPosition(m_targetPosition + increment);
  }

  /**
   * getCurrentPosition - get the current angle of shooter.
   * @return - the current angle of the shooter in degrees
   */
  public double getM_currentPosition() {
    return m_currentPosition; // Relying on periodic to keep currentPosition fresh.
  }

  /**
   * getTargetPosition - get the target angle for the shooter
   * @return - the target angle of shooter degrees.
   */
  public double getM_targetPosition() {
      return m_targetPosition;
  }

  /**
   * getPositionError - get the difference between the current and target angles.
   * @return - the difference between the current and target angles in degrees.
   */
  public double getPositionError() {
    return getM_currentPosition() - getM_targetPosition();
  }

  // Sets the target encoder value.  The PID in the TalonSRX will drive the shooter elevation to this position.
  @Deprecated // This is used when the encoder is wired to the talon.
  public void setSensorPosition(double position) {
    m_shooterElevation.setSelectedSensorPosition(position, 0, 10);
  }

  // Returns true if the shooter elevation is fully lowered.
  // I'd prefer names to be upper and lower but have not made that change.
  public boolean isFwdLimitSwitchClosed() {
    return (m_shooterElevation.isFwdLimitSwitchClosed() == 1 ? true : false);
  }

  // Returns true if the shooter elevation is fully raised.
  // I'd prefer names to be upper and lower but have not made that change.
  public boolean isRevLimitSwitchClosed() {
    return (m_shooterElevation.isRevLimitSwitchClosed() == 1 ? true : false);
  }

  // Returns true if the shooter is at the home position (ie parallel to the ground)
  public boolean isAtHome() {
    return isFwdLimitSwitchClosed();
  }

  @Deprecated
  public void setHomed() {
    m_capturedLimitPosition = true;
    if (isAtHome()) {
      // Remeber that we are homed and set the encoder coordinates to the home position and try to hold it there.
      m_capturedLimitPosition = true;
      m_shooterElevation.setSelectedSensorPosition(ShooterElevationConstants.kShooterElevationHomePosition, Constants.shooterElevationMotor.pidIdx, Constants.shooterElevationMotor.timeout);
      setM_targetPosition(Constants.ShooterElevationConstants.kShooterElevationHomePosition);
    }
  }

  public void unsetHomed() {
    m_capturedLimitPosition = false;
    stop();
  }

  public boolean isHomed() {
    return m_capturedLimitPosition;
  }

  public void readCurrentPosition() {
    m_analogPosition = ShooterElevationConstants.kShooterElevationMaxEncoderVoltage - m_analogInput.getAverageVoltage(); // Intentionally NOT using encoder. dph - 2025-03-03
    m_currentPosition=Utilities.scaleDouble(
      m_analogPosition
      , ShooterElevationConstants.kShooterElevationMinPosition
      , ShooterElevationConstants.kShooterElevationMaxPosition
      , ShooterElevationConstants.kShooterElevationMinEncoderVoltage
      , ShooterElevationConstants.kShooterElevationMaxEncoderVoltage
    ) + 14 ; 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!isHomed()) {
      if (isAtHome()) {
        setHomed();
      }
    }
    
    readCurrentPosition();
    
    if (isHomed()) {
      double error=getPositionError(); 
      m_percentOutput=MathUtil.clamp(
        m_pidController.calculate(error), 
        ShooterElevationConstants.kShooterElevationPeakOutputReverse,
        ShooterElevationConstants.kShooterElevationPeakOutputForward);
      m_shooterElevation.set(ControlMode.PercentOutput, m_percentOutput);
    }
    if (debug) {
      SmartDashboard.putNumber("ShooterElevation Angle",  m_currentPosition);
      SmartDashboard.putNumber("ShooterElevation output", m_percentOutput);
      SmartDashboard.putNumber("ShooterElevation target", m_targetPosition);
      SmartDashboard.putNumber("ShooterElevation analog", m_analogPosition);
      SmartDashboard.putBoolean("ShooterElevation Homed", isHomed());
      SmartDashboard.putBoolean("ShooterElevation fwdsw", isFwdLimitSwitchClosed());
      SmartDashboard.putBoolean("ShooterElevation revsw", isRevLimitSwitchClosed());
    }
  }

  /**
   * onTarget - return true if the angle is within the specified tolerance of the target.
   * @param tolerance
   * @return true if the angle is within the specified tolerance of the target.
   */
  public boolean onTarget(double tolerance) {
    return isHomed() && Math.abs(m_targetPosition - m_currentPosition) <= tolerance;
  }

  /**
   * simulationPeriodic
   */
  @Override
  public void simulationPeriodic() {
    if (!isHomed() && m_homing) {
      m_capturedLimitPosition = true;
    }
    m_currentPosition += m_percentOutput * m_timestep * ShooterElevationConstants.kShooterElevationTimeout;
  }
}
