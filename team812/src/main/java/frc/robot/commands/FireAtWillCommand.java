// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FireAtWillCommand extends Command {
  /** Creates a new FireAtWillCommand. */

  public FireAtWillCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // calculate the target distance and use that to calculate flywheel speed and elevation angle.
    /*
     * The projectile will follow a parabolic path.
     * We can control the intial velocity and the initial angle of projection.
     * We can measure/calculate the distance to the target.
     * We know, a prioi, the height of the target relative to the height of the shooting mechanism.
     * Our intent is to have a specific final velocity at the target.
     * Therefore, using the formula for a parabola we can calculate the andgle and velocity needed.
     */

    double targetHeight = 2.0; // meters, a made up number we would need to get from the field constants.
    double shooterHeight = 1.0; // meters, also made up.  This would be from some robot constants.
    double targetDistance = 4.0; // meters - made up.
    double g = 9.8; // meters/second/second

    /*
     * We want the equation for the parabola the crosses through both the starting and ending position and velocity.
     * Formula for the motion.
     *  y = x * tan(theta) - g*(x**2)/(2*(v**2)cos2(theta))
     */
    double dy = targetHeight - shooterHeight;
    double dx = targetDistance;
    double wheelRadius = Units.inchesToMeters(4.0);
    double wheelCircumference = 2.0 * Math.PI * wheelRadius;

    // Compute a few different solutions just to exercise the code.

    double theta_1500 = fixedFlyWheelSpeedSolution(dx, dy,1500, Units.inchesToMeters(2.0));
    double theta_3000 = fixedFlyWheelSpeedSolution(dx, dy, 3000, Units.inchesToMeters(2.0));
    Map<String, Double> map = fixedAngleAtTargetSolution(dx, dy, wheelRadius, -1);
    Map<String, Double> map2 = fixedAngleAtTargetSolution(dx, dy, wheelRadius, 0.0);
    double rpm = fixedShooterAngleTargetSolution(dx, dy, wheelRadius, Units.degreesToRadians(45));
  }

  /**
   * fixedFlyWheelSpeedSolution - Theta and RPM for fixed flywheel speed
   * @param dxToTarget  (double) - the distance in meters to the target.
   * @param dyToTarget  (double) - the height of the target relative to the shooter.
   * @param rpm         (double) - the rotation speed of the flywheel
   * @param wheelRadius (double) - the radius of the shooting wheels.
   * @return theta      (double) - angle of elevation required to hit the target.
   */
  public double fixedFlyWheelSpeedSolution(
    double dxToTarget,
    double dyToTarget,
    double rpm,
    double wheelRadius) {
    double g = 9.8; // meters/second/second
    double v0 = rpm // (rotation/minute)
              / 60.0 // (1 minute/ 60 seconds)
              * 2.0 * Math.PI * wheelRadius; // meters/rotations
    /*
     * Equations for x and y velocity vs time:
     * y = v0 * sin(theta)*t + 1/2 a * t * t
     * x = v0 * cos(theta)*t 
     * The math on this is hard as it involves a bunch of tricky trig function substitutions.
     * I got the formula from here:
     * https://math.stackexchange.com/questions/3019313/finding-projectile-angle-with-different-elevation-when-velocity-and-range-are-kn
     * 
     * There are always 2 solutions but we will likely always prefer the more direct (lowest) angle.
     * TODO: add a test(s) for flywheel velocity too low to reach the (height,range) of the target
     */
    double theta1 = Math.atan(
      (v0*v0 + Math.sqrt((Math.pow(v0,4) - g*(g*Math.pow(dxToTarget,2) + 2 * dyToTarget * Math.pow(v0,2)))))
      /(g * dxToTarget)
    );
    double theta2 = Math.atan(
      (v0*v0 - Math.sqrt((Math.pow(v0,4) - g*(g*Math.pow(dxToTarget,2) + 2 * dyToTarget * Math.pow(v0,2)))))
      /(g * dxToTarget)
    );
    // I wonder if this handles downward angles properly (ie where the target is below the shooter)
    double theta = Math.abs(theta1) < Math.abs(theta2) ? theta1: theta2; // Pick the shallow angle for the most direct shot.
    
    return theta;
  }

  /**
   * fixedAngleAtTargetSolution
   * 
   * Find the trajectory to the target where rpm and angle are variable but we want the
   * projectile to hit the target at a specific angle.
   * 
   * @param dxToTarget - (double) the x axis distance to the target in meters.
   * @param dyToTarget - (double) the y axis distance to the target in meters.
   * @param wheelRadius - (double) the radius of the flywheel in meters.
   * @param slope - (double) the slope of the trajectory at the target.
   * @return double[2] containing the rpm and theta.
   */
  public Map<String, Double> fixedAngleAtTargetSolution (double h, double w, double wheelRadius, double finalSlope) {
    Map<String, Double> results = new HashMap<>();
    /*
     * solving this time using        A: y = a * X * X + b * x + c;
     * and its derivative             B: y' = 2aX + b;
     * Because our starting position in (0,0) we know c == 0;
     * Refactor B to isolate b we get C: b = y' - 2aX;
     * Replace C into A we get:       D: y = a * X * X + (y' - 2aX);
     * Refactoring C to isolate a     E: a = -(h+w)/(w*w)
     * We can then replace the know a into C to get b.
     * Usin
     */
    double g = 9.8;
    double a = -(h+w)/(w*w);
    double b = finalSlope - 2 * a * w;
    // The vertex is where the derivative == 0 hence:
    double xVertex = -b/(2.0 * a);
    double yVertex = a * xVertex * xVertex + b * xVertex;
    // calculate time knowing that d = 1/2(a)t*t hence t = sqrt(2d/g)
    double tUp = Math.sqrt(2*yVertex/g);   // time from shooter to the vertex
    double tDown = Math.sqrt(2*(yVertex-h)/g); // time from the vertex to the target
    double vx0 = w/(tUp + tDown);
    double vy0 = tUp * g;
    double theta = Math.atan2(vy0, vx0);
    double v0 = Math.sqrt(vx0 * vx0 + vy0 * vy0);
    SmartDashboard.putNumber("shooterAngle", Units.radiansToDegrees(theta));
    SmartDashboard.putNumber("initialVelocity", v0);
    SmartDashboard.putNumber("RPM", v0 * 60.0 / wheelRadius*2.0*Math.PI);
    double yAtTarget = a*w*w + b*w;
    assert yAtTarget == h : "problem in fixedAngleAtTargetSolution";
    double rpm = v0/(wheelRadius*2.0*Math.PI) * 60;
    results.put("RPM", rpm);
    results.put("theta", theta);
    return results;
  }

  /**
   * fixedShooterAngleTargetSolution
   * calculate a flywheel speed to hit the target given a fixed shooting angle on the robot.
   * 
   * @param dxToTarget - (double) x-axis distance to the target in meters.
   * @param dyToTarget - (double) y-axis distance from shooter to the target in meters.
   * @param wheelRadius - (double) the radius of the flywheel in meters.
   * @param theta - (double) the shooting angle in radians where 0 is parallel to the ground.
   * @return (double) the rpm for the flywheel to hit the target.
   */
  public double fixedShooterAngleTargetSolution(
    double dxToTarget,
    double dyToTarget,
    double wheelRadius,
    double theta) {
    /*
     * The trajectory formula being used.:
     *    y = x * tan(theta) - g*(x**2)/(2*(v0**2)cos2(theta))
     * solve for v yields:
     *    v = sqrt(g*x*x/[(y - x * tan(theta))*(2*cos(theta)*cos(theta))]
     */
    double g = 9.8;
    double xTanTheta = dxToTarget * Math.tan(theta);
    double gxx = g * dxToTarget * dxToTarget;
    double cos2theta2 = 2.0 * Math.cos(theta)*Math.cos(theta);
    double v0 = Math.sqrt(-gxx/((cos2theta2)*(dyToTarget - xTanTheta)));
    double yCheck = dxToTarget * Math.tan(theta) - (g*dxToTarget*dxToTarget)/(2.0*v0*v0*Math.cos(theta)*Math.cos(theta));
    double rpm = v0 / (wheelRadius * 2.0 * Math.PI) * 60;
    assert yCheck == dyToTarget : "fixedShooterAngleTargetSolution probably needs a tolerance but should check";
    return rpm;
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
