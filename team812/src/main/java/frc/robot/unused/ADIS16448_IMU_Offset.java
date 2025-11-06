// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.unused;

import edu.wpi.first.wpilibj.ADIS16448_IMU;

/** Add your docs here. */
/*
 * This class extends the WPILIB ADIS16448 to add an offset to the gyro value to make it easier to align the drivetrain pose to the estimated pose.
 */
public class ADIS16448_IMU_Offset extends ADIS16448_IMU {
    private double m_angleOffset = 0.0;  // This is in degrees
    public double getAngle() {
        return super.getAngle() + m_angleOffset;

    }
    public double setAngle(double desiredAngle) {
        // Not sure if I need to worry about the range, 0..2Pi or -pi..pi etc.
        m_angleOffset = desiredAngle - super.getAngle();
        return super.getAngle() + m_angleOffset;
    }
    public void reset() {
        m_angleOffset = 0.0;
        super.reset();
    }

}
