// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AllianceConfigurationSubsystem;
import frc.robot.subsystems.DriveSubsystemSRX;

/**
 * This comand drives the robot the specified distance
 * If controlRotation is set, it will try to hold the angle in the move pose.
 * If controlRotation is set, it will just manage the X,Y of the move.
 * All angles are in radians.
 */
// TODO: convert to extend GotoPoseCommand
public class RotateRobotCommand extends Command {

  public class RotateRobotConfig {
    
    private double maxRotation;
    private double angularP;
    private double angularI;
    private double angularD;
    private double angularF;
    private double angularIZone;
    private double angularTolerance;

    /**
     * default constructor
     */
    public RotateRobotConfig() {
      maxRotation = 0.8;
      angularP = 1.0;
      angularI = angularP/100.0; // angularI/100.0;
      angularD = angularP/30.0;
      angularF = 0.0;
      angularIZone = Units.degreesToRadians(5.0);
      angularTolerance = Units.degreesToRadians(1.0);
    }

    public RotateRobotConfig setMaxRotation(double maxRotation) {this.maxRotation = maxRotation; return this; };
    public RotateRobotConfig setAngularP(double angularP) {this.angularP = angularP; return this; };
    public RotateRobotConfig setAngularI(double angularI) {this.angularI = angularI; return this; };
    public RotateRobotConfig setAngularD(double angularD) {this.angularD = angularD; return this; };
    public RotateRobotConfig setAngularF(double angularF) {this.angularF = angularF; return this; };
    public RotateRobotConfig setAngularIZone(double angularIZone) {this.angularIZone = angularIZone; return this; };
    public RotateRobotConfig setAngularTolerance(double angularTolerance) {this.angularTolerance = angularTolerance; return this; };
    
    public double getMaxRotation() { return maxRotation; }
    public double getAngularP() { return angularP; }
    public double getAngularI() { return angularI; }
    public double getAngularD() { return angularD; }
    public double getAngularF() { return angularF; }
    public double getAngularIZone() { return angularIZone; }
    public double getAngularTolerance() { return angularTolerance; }
  } // RotateRobotConfig Class

  private final DriveSubsystemSRX m_robotDrive;
  private final double m_theta;
  private final boolean m_relative;
  private final RotateRobotConfig m_config;
  private double m_targetTheta;
  private boolean debug = true;
  private Timer m_timer = new Timer();
  
  private PIDController m_thetaController;
  private boolean m_onTarget;

  /** Creates a new DriveDistanceCommand. */
  public RotateRobotCommand(DriveSubsystemSRX robotDrive, double theta, boolean relative) {
    m_robotDrive = robotDrive;
    m_theta = theta;
    m_relative = relative;
    m_config = new RotateRobotConfig();

    // Set up the PID controller.
    m_thetaController = new PIDController(m_config.getAngularP(), m_config.getAngularI(), m_config.getAngularD());
    m_thetaController.setIZone(m_config.getAngularIZone());
    m_thetaController.setTolerance(m_config.getAngularTolerance()); // did not work, dont understand yet
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI); // Tell PID Controller to expect inputs between -180 and 180 degrees (in Radians). // NEW 2/1/2024
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(robotDrive);
;  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // The timer is used for tuning the PID parameters.
    if (debug) {
      m_timer.reset();
      m_timer.start();
    }
    
    // Calculate the target angle using absolute or relative depending on <relative>.
    // Ensure that the target is between -pi and pi to avoid rotating the long
    // way around.
    if (m_relative) {
      // get the robot's current rotation from the drivetrain
      double startingTheta = m_robotDrive.getPose().getRotation().getRadians(); 
      m_targetTheta = MathUtil.angleModulus(startingTheta + m_theta);
    } else {
      // set the target angle to the angle specified in the command.
      m_targetTheta = AllianceConfigurationSubsystem.allianceAdjustedTelopRotation(m_theta);
    }
    
    m_onTarget = false;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotationError = 0.0;
    double currentTheta;
    double xSpeed = 0.0;
    double ySpeed = 0.0;
    double rotationSpeed = 0.0;

    currentTheta = m_robotDrive.getPose().getRotation().getRadians();
    if (debug) SmartDashboard.putNumber("RR theta", currentTheta);
    if (debug) SmartDashboard.putNumber("RR target", m_targetTheta);
    
    rotationError = MathUtil.inputModulus(m_targetTheta - currentTheta, -Math.PI, Math.PI);
    if (debug) SmartDashboard.putNumber("RR Error", Units.radiansToDegrees(rotationError));
    
    // Test to see if we have arrived at the requested angle within the specified tolerance.
    // Need to also test for velocity, otherwise momentum could send us past the goal.
      SmartDashboard. putNumber ("RRConfig", Units.radiansToDegrees( m_config.getAngularTolerance()));
    if (Math.abs(rotationError) < m_config.getAngularTolerance()) {
      // Yes, we have arrived
      xSpeed = 0.0;
      ySpeed = 0.0;
      rotationSpeed = 0.0;
      SmartDashboard.putString ("RR Test", "onTarget"); 
      m_onTarget = true;
    } else {
      rotationSpeed = MathUtil.clamp(m_thetaController.calculate(rotationError, 0),-m_config.getMaxRotation(), m_config.getMaxRotation());
     SmartDashboard.putString ("RR Tes  t", "onMark"); 
      m_onTarget = false;
    }
    if (debug) SmartDashboard.putBoolean("RR OnTarget", m_onTarget);
    if (debug) SmartDashboard.putNumber("RR rSpeed", rotationSpeed);
    m_robotDrive.drive(-xSpeed, -ySpeed, -rotationSpeed, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_robotDrive.drive(0, 0, 0, true, true);
    if (debug) SmartDashboard.putNumber("RR time", m_timer.get()); // Display how long the rotation took.
    if (debug) m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_onTarget;
  }
}
