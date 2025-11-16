// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utilities;
import frc.robot.subsystems.AllianceConfigurationSubsystem;
import frc.robot.subsystems.DriveSubsystemSRX;

/**
 * This comand drives the robot the specified distance
 * If controlRotation is set, it will try to hold the angle in the move pose.
 * If controlRotation is set, it will just manage the X,Y of the move.
 * All angles are in radians.
 */
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
      angularP = 0.35;
      angularI = 0; // angularI/100.0;
      angularD = 0.0; // angularP*10.0;
      angularF = 0.0;
      angularIZone = Units.degreesToRadians(10.0);
      angularTolerance = Units.degreesToRadians(5.0);
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

  private final DriveSubsystemSRX robotDrive;
  private final double theta;
  private final boolean relative;
  private final RotateRobotConfig config;
  private double startingTheta;
  private double targetTheta;
  private boolean debug = true;
  
  private PIDController rotationController;
  private boolean onTarget;

  /** Creates a new DriveDistanceCommand. */
  public RotateRobotCommand(DriveSubsystemSRX robotDrive, double theta, boolean relative) {
    this.robotDrive = robotDrive;
    this.theta = theta;
    this.relative = relative;
    this.config = new RotateRobotConfig();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(robotDrive);
;  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //debugPID = (RobotContainer.m_BlackBox.isSwitchRight());

    double angularP = config.getAngularP();
    double angularI = config.getAngularI();

    /*
    if (debugPID) {
      debugIterations = 0;
      //config.setAngularTolerance(Units.degreesToRadians(0.1)); // set a smaller window for success.
      angularP = RobotContainer.m_BlackBox.getPotValueScaled(OIConstants.kControlBoxPotX, 0.0, 1.0);
      angularI = RobotContainer.m_BlackBox.getPotValueScaled(OIConstants.kControlBoxPotY, 0.0, 0.001);
      SmartDashboard.putNumber("BB P", angularP);
      SmartDashboard.putNumber("BB I", angularI);
    }
    */
    
    // get the robot's current rotation from the drivetrain
    startingTheta = robotDrive.getPose().getRotation().getRadians(); 

    // Calculate the target angle using absolute or relative depending on <relative>.
    // Ensure that the target is between -pi and pi to avoid rotating the long
    // way around.
    if (relative) {
      targetTheta = MathUtil.inputModulus(startingTheta + theta, -Math.PI, Math.PI);
      
    } else {
      if (AllianceConfigurationSubsystem.isBlueAlliance()){
         targetTheta =  MathUtil.inputModulus(theta, -Math.PI, Math.PI);
         if (debug) SmartDashboard.putNumber("RR 1", targetTheta);
      } else {
           targetTheta =  MathUtil.inputModulus(theta +Math.PI, -Math.PI, Math.PI); 
           if (debug) SmartDashboard.putNumber("RR 2", targetTheta);
      }
    }
    rotationController = new PIDController(angularP, angularI, config.getAngularD());
    rotationController.setIZone(config.getAngularIZone());
    rotationController.setTolerance(config.getAngularTolerance()); // did not work, dont understand yet
    rotationController.enableContinuousInput(-Math.PI, Math.PI); // Tell PID Controller to expect inputs between -180 and 180 degrees (in Radians). // NEW 2/1/2024
    onTarget = false;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotationError = 0.0;
    double currentTheta;
    double xSpeed = 0.0;
    double ySpeed = 0.0;
    double rotationSpeed = 0.0;

    currentTheta = robotDrive.getPose().getRotation().getRadians();
    if (debug) SmartDashboard.putNumber("RR theta", currentTheta);
    if (debug) SmartDashboard.putNumber("RR target", targetTheta);
    
    rotationError = MathUtil.inputModulus(targetTheta - currentTheta, -Math.PI, Math.PI);
    if (debug) SmartDashboard.putNumber("RR Error", Units.radiansToDegrees(rotationError));
    
    // Test to see if we have arrived at the requested angle within the specified tolerance.
    // Need to also test for velocity, otherwise momentum could send us past the goal.
      SmartDashboard. putNumber ("RRConfig", Units.radiansToDegrees( config.getAngularTolerance()));
    if (Math.abs(rotationError) < config.getAngularTolerance()) {
      // Yes, we have arrived
      xSpeed = 0.0;
      ySpeed = 0.0;
      rotationSpeed = 0.0;
      SmartDashboard.putString ("RR Test", "onTarget"); 
      onTarget = true;
    } else {
      rotationSpeed = MathUtil.clamp(rotationController.calculate(rotationError, 0),-config.getMaxRotation(), config.getMaxRotation());
     SmartDashboard.putString ("RR Tes  t", "onMark"); 
      onTarget = false;
    }
    if (debug) SmartDashboard.putBoolean("RR OnTarget", onTarget);
    if (debug) SmartDashboard.putNumber("RR rSpeed", rotationSpeed);
    robotDrive.drive(-xSpeed, -ySpeed, -rotationSpeed, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    robotDrive.drive(0, 0, 0, true, true); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return onTarget;
  }
}
