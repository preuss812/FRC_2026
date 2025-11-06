// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ModifiedSlewRateLimiter;
import frc.robot.Utilities;
import frc.utils.DrivingConfig;
import frc.utils.PreussDriveSimulation;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;


public class DriveSubsystemSRX extends SubsystemBase {
  // Create MAXSRXSwerveModules
  private final MAXSRXSwerveModule m_frontLeft = new MAXSRXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftTurningEncoderCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSRXSwerveModule m_frontRight = new MAXSRXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightTurningEncoderCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSRXSwerveModule m_rearLeft = new MAXSRXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kRearLeftTurningEncoderCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSRXSwerveModule m_rearRight = new MAXSRXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kRearRightTurningEncoderCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SerialPort.Port.kUSB1);


  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentXSpeed = 0.0;
  private double m_currentYSpeed = 0.0;

  private ModifiedSlewRateLimiter m_xSpeedLimiter;
  private ModifiedSlewRateLimiter m_ySpeedLimiter;
  private ModifiedSlewRateLimiter m_rotLimiter;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(-m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  public enum DrivingMode {
    SPEED,
    PRECISION
  }
  // Default driving mode and associated constants to SPEED mode.
  private DrivingMode drivingMode;
  private double maxSpeedMetersPerSecond;
  private double maxAngularSpeed;
  public DrivingConfig defaultAutoConfig;
  public DrivingConfig debugAutoConfig;
  public DrivingConfig circleAutoConfig;


  private final boolean debug = true;

  /** Creates a new DriveSubsystemSRXSRX. */
  public DriveSubsystemSRX() {
    setDrivingMode(DrivingMode.SPEED); // Default driving mode is speed mode.

    // set up some known-good PID configurations for automatic and semi-automatic driving.
    defaultAutoConfig = new DrivingConfig()
    .setMaxThrottle(0.8)
    .setMaxRotation(0.8)
    .setLinearP(0.25)
    .setLinearIZone(Units.inchesToMeters(4.0))
    .setLinearTolerance(Units.inchesToMeters(2.0))
    .setAngularP(0.5)
    .setAngularIZone(Units.degreesToRadians(10.0))
    .setAngularTolerance(Units.degreesToRadians(2.0));

    debugAutoConfig = new DrivingConfig()
      .setMaxThrottle(0.2)
      .setMaxRotation(0.2)
      .setLinearP(0.1)
      .setLinearIZone(Units.inchesToMeters(4.0))
      .setLinearTolerance(Units.inchesToMeters(2.0))
      .setAngularP(0.5)
      .setAngularIZone(Units.degreesToRadians(10.0))
      .setAngularTolerance(Units.degreesToRadians(2.0));
      
      circleAutoConfig = new DrivingConfig()
      .setMaxThrottle(0.8)
      .setMaxRotation(0.8)
      .setLinearP(10.0)
      .setLinearIZone(Units.inchesToMeters(4.0))
      .setLinearTolerance(Units.inchesToMeters(2.0))
      .setAngularP(0.5)
      .setAngularIZone(Units.degreesToRadians(10.0))
      .setAngularTolerance(Units.degreesToRadians(2.0));

  circleAutoConfig = new DrivingConfig()
    .setMaxThrottle(0.8)
    .setMaxRotation(0.8)
    .setLinearP(1.00)
    .setLinearIZone(Units.inchesToMeters(4.0))
    .setLinearTolerance(Units.inchesToMeters(0.0))
    .setAngularP(0.5)
    .setAngularIZone(Units.degreesToRadians(10.0))
    .setAngularTolerance(Units.degreesToRadians(2.0));

    // TODO Do we need to reset the gyro here?
  }
// TODO: This seems redundant to the code below in periodic.  Perhaps should refactor.
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        };
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    if (debug) {
      SmartDashboard.putNumber("gyro_angle", MathUtil.inputModulus(m_gyro.getAngle(), -180, 180));
      Utilities.toSmartDashboard("DriveTrain", this.getPose()); 
      SmartDashboard.putNumber("Robot X", this.getPose().getX()); 
      SmartDashboard.putNumber("Robot Y", this.getPose().getY()); 
    }

    m_odometry.update(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }
  /**
   * Method to drive the robottaking into account the alliance color.
   * The drive subsystem and swerve module are using the blue alliance field coordinate system.
   * That means that possitive x is moving away from the blue alliance wall and driver station.
   * When we are the red alliance we want the robot to move away from the red alliance wall and driver station.
   * This method will rotate the joystick inputs 180 degrees if we are in the red alliance.
   * @param xSpeed - the x joystick input -1.0..1.0
   * @param ySpeed - the y joystick input -1.0..1.0
   * @param rot    - the rotation joystick input -1.0..1.0
   * @param fieldRelative - true/false if the x and y coordinates are relative to the field (true) or relative to the robot (false)
   * @param rateLimit - true/false if the slew rate limiting is enabled.
   */
  public void allianceRelativeDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
      
      if (Utilities.isBlueAlliance())
        drive(xSpeed, ySpeed, rot, fieldRelative, rateLimit); // The coordinates are fine if the 
      else
        drive(-xSpeed, -ySpeed, rot, fieldRelative, rateLimit); // Rotate the joystick inputs 180 degrees if we are on the Red Alliance.
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.  Only true is tested.
   * @param rateLimit     Whether to enable rate limiting for smoother control. True is recommended.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    // TODO this rate limiting is using the wrong units.  This should be performed on the x,y,rot delivered values
    // to align the units to the constants.  This is a bug.
    if (rateLimit) {
      // In this mode thte x, y, and rotation inputs are treated independently.
      // This is the same as putting slew rate limiters on the joystick outputs.
      m_currentXSpeed   = m_xSpeedLimiter.calculate(xSpeed);
      m_currentYSpeed   = m_ySpeedLimiter.calculate(ySpeed);
      m_currentRotation = m_rotLimiter.calculate(rot);
    } else {
      // In this mode, the inputs are passed directly to the Swerrve module without any slew rate limiting.
      // This is the most responsive mode but can be difficult to control.
      m_currentXSpeed = xSpeed;
      m_currentYSpeed = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units (meters/sec and radians/sec) for the drivetrain
    double xSpeedDelivered = m_currentXSpeed * maxSpeedMetersPerSecond;
    double ySpeedDelivered = m_currentYSpeed * maxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * maxAngularSpeed;

    /*
     * For simulation, update the position of the robot based on the requested speeds
     */
    if (RobotContainer.isSimulation()) {
      RobotContainer.m_preussDriveSimulation.drive(xSpeedDelivered, ySpeedDelivered, rotDelivered);

    }
    // Convert the inputs into chassis speeds, ie speeds and rotations for each wheel.
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(-m_gyro.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
       
    // Enforce the maximum speed of the robot
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, maxSpeedMetersPerSecond);
    
    // Set the swerve module states
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the wheels to point straight ahead.
   */
  public void wheelsStraightAhead() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   * I think this is the wrong one and setX is the one that should be used.
   */
  public void wheels45() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, maxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle()).getDegrees();
  }

  /**
   * Returns the rotation of the robot.
   *
   * @return the robot's rotation as Rotation2d.
   */
  public Rotation2d getRotation() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
  /**
   * Reset the gyro angle as specified, presumably to align the drivetrain to the field.
   * @param desiredngle
   * @return
   */
  public double setAngleDegrees(double desiredAngle) {
    // There is something wrong here as the results are 180 out.
    if (debug) {
      SmartDashboard.putNumber("SetAngle", desiredAngle); // minus but we should have added m_gyro.inverted instead
      SmartDashboard.putNumber("SetAngleGyro", m_gyro.getAngle());
      SmartDashboard.putNumber("SetAngleOldAdj", m_gyro.getAngleAdjustment());
      SmartDashboard.putNumber("SetAngleNewAdj", desiredAngle - (m_gyro.getAngle() - m_gyro.getAngleAdjustment()));
    }
    m_gyro.setAngleAdjustment((desiredAngle - (m_gyro.getAngle() - m_gyro.getAngleAdjustment())));    
    if (debug) SmartDashboard.putNumber("SetAngleNew", m_gyro.getAngle());

    return m_gyro.getAngle();  // Return the new angle for chaining
  }

  public void quiesce() {
    m_frontLeft.quiesce();
    m_frontRight.quiesce();
    m_rearLeft.quiesce();
    m_rearRight.quiesce();
  }

  /**
   * setDrivingMode - Select the driving mode for the robot.  The driving mode is either SPEED or PRECISION.
   * in PRECISION mode the robot will drive and accelerate more slowly and with more control.
   * @param drivingMode - SPEED or PRECISION
   * @return the previous drivingMode
   */
  public DrivingMode setDrivingMode(DrivingMode drivingMode) {
    DrivingMode result = this.drivingMode;
    this.drivingMode = drivingMode;
    if (drivingMode == DrivingMode.SPEED) {
      maxSpeedMetersPerSecond = DriveConstants.kMaxSpeedMetersPerSecond;
      maxAngularSpeed = DriveConstants.kMaxAngularSpeed;
      
      m_rotLimiter    = new ModifiedSlewRateLimiter(DriveConstants.kRotationalIncreaseSlewRate, DriveConstants.kRotationalDecreaseSlewRate, 0);
      m_xSpeedLimiter = new ModifiedSlewRateLimiter(DriveConstants.kMagnitudeIncreaseSlewRate,  DriveConstants.kMagnitudeDecreaseSlewRate,  0);
      m_ySpeedLimiter = new ModifiedSlewRateLimiter(DriveConstants.kMagnitudeIncreaseSlewRate,  DriveConstants.kMagnitudeDecreaseSlewRate,  0);

      SmartDashboard.putString("DriveMode", "SPEED");
    } else /* if (drivingMode == DrivingMode.PRECISION) */ {
      maxSpeedMetersPerSecond = DriveConstants.kMaxSpeedMetersPerSecondPM;
      maxAngularSpeed = DriveConstants.kMaxAngularSpeedPM;
      
      m_rotLimiter    = new ModifiedSlewRateLimiter(DriveConstants.kRotationalIncreaseSlewRatePM, DriveConstants.kRotationalDecreaseSlewRatePM, 0);
      m_xSpeedLimiter = new ModifiedSlewRateLimiter(DriveConstants.kMagnitudeIncreaseSlewRatePM,  DriveConstants.kMagnitudeDecreaseSlewRatePM,  0);
      m_ySpeedLimiter = new ModifiedSlewRateLimiter(DriveConstants.kMagnitudeIncreaseSlewRatePM,  DriveConstants.kMagnitudeDecreaseSlewRatePM,  0);
      SmartDashboard.putString("DriveMode", "PRECISION");
    }
    return result;
  }
  
  /**
   * getDrivingMode
   * @return the current drivingMode either SPEED or PRECISION
   */
  public DrivingMode getDrivingMode() {
    return drivingMode;
  }
  
  public void followTrajectory(SwerveSample sample) {
    // Get the current pose of the robot
    Pose2d pose = getPose();
    pose = RobotContainer.m_preussDriveSimulation.getCurrentPose();
    PIDController xController = new PIDController(10.0, 0.0, 0.0);
    PIDController yController = new PIDController(10.0, 0.0, 0.0);
    PIDController headingController = new PIDController(7.5, 0.0, 0.0);
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    var x = xController.calculate(pose.getX(), sample.x);
    System.out.println(x);
    // Generate the next speeds for the robot
    ChassisSpeeds speeds = new ChassisSpeeds(
      sample.vx + xController.calculate(pose.getX(), sample.x),
      sample.vy + yController.calculate(pose.getY(), sample.y),
      sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
    );
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    if (RobotContainer.isSimulation())
      RobotContainer.m_preussDriveSimulation.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);

    // Enforce the maximum speed of the robot
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, maxSpeedMetersPerSecond);
    
    // Set the swerve module states
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

}
