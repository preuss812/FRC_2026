// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    private final TalonSRX master;
    private final TalonSRX follower;

    // Adjust for your physical elevator
    private static final int MASTER_ID = 1;
    private static final int FOLLOWER_ID = 2;

    // PID values (tune these!)
    private static final double kP = 0.25;
    private static final double kI = 0.0;
    private static final double kD = 2.0;

    // Example motion profile settings
    private static final int CRUISE_VELOCITY = 15000;  // encoder units / 100ms
    private static final int ACCELERATION = 6000;      // encoder units / 100ms/sec

    public ElevatorSubsystem() {
        master = new TalonSRX(MASTER_ID);
        follower = new TalonSRX(FOLLOWER_ID);

        // Reset to factory defaults
        master.configFactoryDefault();
        follower.configFactoryDefault();

        // Master encoder setup
        master.configSelectedFeedbackSensor(
            TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative,
            0, 20);

        // Follower follows master
        follower.follow(master);

        // Motor inversion as needed
        master.setInverted(false);
        follower.setInverted(false);

        // Brake mode (recommended for elevators)
        master.setNeutralMode(NeutralMode.Brake);
        follower.setNeutralMode(NeutralMode.Brake);

        // PIDF config
        master.config_kP(0, kP);
        master.config_kI(0, kI);
        master.config_kD(0, kD);

        // Motion Magic config
        master.configMotionCruiseVelocity(CRUISE_VELOCITY);
        master.configMotionAcceleration(ACCELERATION);

        // Optional soft limits (example)
        master.configForwardSoftLimitThreshold(50000);
        master.configForwardSoftLimitEnable(true);
        master.configReverseSoftLimitThreshold(0);
        master.configReverseSoftLimitEnable(true);

        // Configure current limits to protect the motors
        master.configContinuousCurrentLimit(30); // Amps
        master.configPeakCurrentLimit(40);      // Amps
        master.configPeakCurrentDuration(100);  // Milliseconds
        master.enableCurrentLimit(true);
    }

    /** Move elevator using percent output (manual override). */
    public void setPercent(double speed) {
        master.set(ControlMode.PercentOutput, speed);
    }

    /** Move elevator to a position using MotionMagic. */
    public void setPosition(int targetTicks) {
        master.set(ControlMode.MotionMagic, targetTicks);
    }

    /** Stop motors. */
    public void stop() {
        master.set(ControlMode.PercentOutput, 0);
    }

    /** Get current encoder position. */
    public int getPosition() {
        return (int)master.getSelectedSensorPosition();
    }

    /** Reset the encoder position to zero. */
    public void resetEncoder() {
        master.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {
        // Add telemetry for debugging
        SmartDashboard.putNumber("Elevator Position", getPosition());
        SmartDashboard.putNumber("Elevator Output Percent", master.getMotorOutputPercent());
        SmartDashboard.putNumber("Elevator Current", master.getStatorCurrent());
    }
}
