// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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


    /*
     * if the velocity of the flywheel is fixed and if we want to enter
     * the target at the top of the arc
     * we can first solve for the time it takes for the y axis
     * velocity to go to 0 and then using that time we can 
     * compute the x initial velocity and solve for the angle needed.
     */
   
    // The intial velocity v0 = sqrt(vx0^2 + vy0^2)
    // Note: vx is constant.
    // find the time for the projectile to slow fron initial velocity to 0.
    //       vy = vy0 - g * t;   where t = time in seconds, g = gravity.  v = a*t;
    //       dy = 1/2 * g * t^2;
    //  =>   t = sqrt(dy*2/g);    // solve for t
    //  => vy0 = t * g;
    //       
    double t = Math.sqrt(dy/(0.5 * g));
    // Now that we have time we can solve for vx since we are ignoring wind resistance etc.
    double vx0 = dx / t;
    // With time we can also solve for vy0 given time and gravity.
    double vy0 = g * t;
    // Now that we have vx0 and vy0 we can calculate total initial initial angle and velocity
    double theta = Math.atan2(vy0, vx0);
    double v0 = Math.sqrt(vx0*vx0 + vy0*vy0);
    SmartDashboard.putNumber("shooterT", t);
    SmartDashboard.putNumber("shooterTheta", theta);
    SmartDashboard.putNumber("shooterAngle", Units.radiansToDegrees(theta));
    SmartDashboard.putNumber("initialVelocity", v0);
    SmartDashboard.putNumber("RPM", v0 * 60.0 / wheelCircumference);


    /*
     * Alternatively, look for a solution where the elevation angle is constant and 
     * we can look for the velocity needed to hit the target.
     * recall:
     *    y = x * tan(theta) - g*(x**2)/(2*(v**2)cos2(theta))
     * solve for v (which is v0) yields:
     *    v = sqrt(g*x*x/[(y - x * tan(theta))*(2*cos(theta)*cos(theta))]
     */
    double omega = Units.degreesToRadians(45); // A number for testing
    double xTanOmega = dx * Math.tan(omega);
    double gxx = g * dx * dx;
    double cos2omega2 = 2.0 * Math.cos(omega)*Math.cos(omega);
    double v = Math.sqrt(-gxx/((cos2omega2)*(dy - xTanOmega)));
    SmartDashboard.putNumber("shooterAngle2", Units.radiansToDegrees(omega));
    SmartDashboard.putNumber("initialVelocity2", v);
    double yCheck = dx * Math.tan(omega) - (g*dx*dx)/(2.0*v*v*Math.cos(omega)*Math.cos(omega));
    SmartDashboard.putBoolean("yCheck", yCheck == dy);  // ycheck should == dy
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
