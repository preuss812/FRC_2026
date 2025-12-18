// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.preuss.sensors;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;

/**
 * AHRSInvertable
 * This class extends the AHRS class to allow inverting of the gyro and adds some
 * convenience functions and capabilities like keeping the angles between -180 to 180 degrees.
 */
public class AHRSInvertable extends AHRS {
    private boolean m_isInverted = false;

    /*
     * AHRSInvertable - Constructs the AHRS 
     * @param serial_port_id the port the gyro is connected to.
     * @return the newly constructed object.
     */
    public AHRSInvertable(SerialPort.Port serial_port_id) {
        super(serial_port_id);
    }

    /**
     * setInverted - indicate whether the gyro is inverted or not.  The default is not inverted.
     * @param isInverted - The state of inversion true is inverted.
     * @return the AHRS object.
     */
     public AHRSInvertable setInverted(boolean isInverted) {
        m_isInverted = isInverted;
        return this;
    }

    /*
     * isInverted - get the current inversion status, true = inverted.
     * @return the inversion state where true is inverted.
     */
    public boolean isInverted() {
        return m_isInverted;
    }

    /**
     * setAngleAdjustment - set the offset to be added to the raw angle when getAngle is called.
     * @param adjustment - the adjustment in degrees to be added to the raw angle (yaw) measurement.
     */
    @Override
    public void setAngleAdjustment(double adjustment) {
        if (m_isInverted) {
            super.setAngleAdjustment(-adjustment);
        } else {
            super.setAngleAdjustment(adjustment);
        }
    }

    /**
     * getAngleAdjustment - get the current adjustment to the raw angle for getAngle() results.
     * @return - the current adjustment.
     */
    @Override
    public double getAngleAdjustment() {
    	double angleAdjustment;
        if (m_isInverted) {
            angleAdjustment = -super.getAngleAdjustment();
        } else {
            angleAdjustment = super.getAngleAdjustment();
        }
        return angleAdjustment;
    }

    /**
     * setAngle - set the angle adjustment for the gyro to report the specified angle.
     * @param - the desired angle in degrees.
     */
    public double setAngle(double desiredAngle) {
        double adjustment = desiredAngle - (getAngle() - getAngleAdjustment());
        adjustment = MathUtil.inputModulus(adjustment, -180.0, 180.0);
        setAngleAdjustment(adjustment);
        double resultingAngle = getAngle();
        if (MathUtil.inputModulus(desiredAngle, -180.0, 180.0) - resultingAngle > 1e-6) {
            System.out.println("These should match!");
        }
        return resultingAngle;
    }

    /**
     * getAngle - get the current gyro measurement in degrees
     * @return the adjusted gyro measurement in degrees.
     */
    @Override
    public double getAngle() {
        double angle;
        if (m_isInverted) {
            angle = -super.getAngle();
        } else {
            angle = super.getAngle();
        }
        return MathUtil.inputModulus(angle, -180.0, 180.0);
    }

    /**
     * getRotation - get the current gyro measurement as a Rotation2d
     * @return the current gyro measurement as a Rotation2d.
     */
    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(getAngle());
    }

    /**
     * getDegrees - get the current gyro measurement in degrees
     * @return - the current gyro measurement in degrees.
     */
    public double getDegrees() {
        return getAngle();
    }

    /**
     * iterate - simulate robot rotation
     * @param omegaRadiansPerSecond - (double) the current rotation speed of the robot in radians/second.
     * @param timestep - (double) the simulation interval in seconds.
     */
    public void iterate(double omegaRadiansPerSecond, double timestep) {
        double dTheta = Units.radiansToDegrees(omegaRadiansPerSecond * timestep);
        setAngle(getAngle() + dTheta);
    }
}
