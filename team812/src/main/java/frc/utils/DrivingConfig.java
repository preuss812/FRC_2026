package frc.utils;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.DriveSubsystemSRX.DrivingMode;

/**
 * set up commonly used driving controls
 */
public class DrivingConfig {
    private DrivingMode drivingMode; // Either PRECISON or SPEED
    private double maxThrottle;         // Maximum throttle in percent. (ie 0..1)
    private double linearP;             // PID P term for X,Y motion. Effectively percent output per meter from target.
    private double linearI;             // PID I term for X,Y motion.
    private double linearD;             // PID D term for X,Y motion.
    private double linearF;             // PID Feed forward for X,Y motion.
    private double linearIZone;         // Range around goal in meters to accumulate I errors.
    private double linearTolerance;     // Tolerance in meters to consider the robot on target.

    private double maxRotation;         // maximum rotation input in precent (ie 0..1).
    private double angularP;            // PID P term for rotation in radians.
    private double angularI;            // PID I term for rotation
    private double angularD;            // PID D term for rotation
    private double angularF;            // PID Feed forward for rotation.
    private double angularIZone;        // Range around goal in radians to accumulate I errors
    private double angularTolerance;    // Tolereance in radians to consider on target.

    /**
     * default constructor
     */
    public DrivingConfig() {
      // Set up default values.  These should be good for actual driving.
      drivingMode = DrivingMode.SPEED;
      maxThrottle = 0.80;
      linearP = 1.0;
      linearI = 0.0; // linearP/100.0;
      linearD = 0.0; // linearP*10.0;
      linearF = 0.0;
      linearIZone = Units.inchesToMeters(4.0);
      linearTolerance = Units.inchesToMeters(2.0);

      maxRotation = 0.8;
      angularP =  0.35;
      angularI = 0.0; // angularI/100.0;
      angularD = 0.0; //angularP*10.0;
      angularF = 0.0;
      angularIZone = Units.degreesToRadians(10.0);
      angularTolerance = Units.degreesToRadians(5.0);
    }
    public DrivingConfig(DrivingConfig original) {
        this.drivingMode = original.drivingMode;
        this.maxThrottle = original.maxThrottle;
        this.linearP = original.linearP;
        this.linearI = original.linearI;
        this.linearD = original.linearD;
        this.linearF = original.linearF;
        this.linearIZone = original.linearIZone;
        this.linearTolerance = original.linearTolerance;

        this.maxRotation = original.maxRotation;
        this.angularP = original.angularP;
        this.angularI = original.angularI;
        this.angularD = original.angularD;
        this.angularF = original.angularF;
        this.angularIZone = original.angularIZone;
        this.angularTolerance = original.angularTolerance;

    }
    public DrivingConfig setDrivingMode(DrivingMode drivingMode) { this.drivingMode = drivingMode; return this; };
    public DrivingConfig setMaxThrottle(double maxThrottle) {this.maxThrottle = maxThrottle; return this; };
    public DrivingConfig setLinearP(double linearP) {this.linearP = linearP; return this; };
    public DrivingConfig setLinearI(double linearI) {this.linearI = linearI; return this; };
    public DrivingConfig setLinearD(double linearD) {this.linearD = linearD; return this; };
    public DrivingConfig setLinearF(double linearF) {this.linearF = linearF; return this; };
    public DrivingConfig setLinearIZone(double linearIZone) {this.linearIZone = linearIZone; return this; };
    public DrivingConfig setLinearTolerance(double linearTolerance) {this.linearTolerance = linearTolerance; return this; };

    public DrivingConfig setMaxRotation(double maxRotation) {this.maxRotation = maxRotation; return this; };
    public DrivingConfig setAngularP(double angularP) {this.angularP = angularP; return this; };
    public DrivingConfig setAngularI(double angularI) {this.angularI = angularI; return this; };
    public DrivingConfig setAngularD(double angularD) {this.angularD = angularD; return this; };
    public DrivingConfig setAngularF(double angularF) {this.angularF = angularF; return this; };
    public DrivingConfig setAngularIZone(double angularIZone) {this.angularIZone = angularIZone; return this; };
    public DrivingConfig setAngularTolerance(double angularTolerance) {this.angularTolerance = angularTolerance; return this; };

    public DrivingMode getDrivingMode() { return drivingMode; }
    public double getMaxThrottle() { return maxThrottle; }
    public double getLinearP() { return linearP; }
    public double getLinearI() { return linearI; }
    public double getLinearD() { return linearD; }
    public double getLinearF() { return linearF; }
    public double getLinearIZone() { return linearIZone; }
    public double getLinearTolerance() { return linearTolerance; }
    
    public double getMaxRotation() { return maxRotation; }
    public double getAngularP() { return angularP; }
    public double getAngularI() { return angularI; }
    public double getAngularD() { return angularD; }
    public double getAngularF() { return angularF; }
    public double getAngularIZone() { return angularIZone; }
    public double getAngularTolerance() { return angularTolerance; }

    public DrivingConfig scaleSpeed(DrivingMode drivingMode, double scaleFactor) {
        DrivingConfig result = new DrivingConfig(this);

        return result.setDrivingMode(drivingMode)
        .setMaxThrottle(this.getMaxThrottle()*scaleFactor)
        .setMaxRotation(this.getMaxRotation()*scaleFactor)
        .setLinearP(this.getLinearP() * scaleFactor)
        .setAngularP(this.getAngularP() * scaleFactor);
    }
} // DrivingConfig Class
