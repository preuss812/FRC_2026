package frc.utils;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * Create a TalonSRX motor object using config
 */
public class PreussMotor extends WPI_TalonSRX{
    public PreussMotor(PreussMotorConfig config) {
        // Note: we are not selecting magic motion or any other control mode.
        // Note: does not handle follower motors.
        // Feel free to add additional config settings here and in PreussMotorConfig.
        // Set the default to match the values provided by setting factory defaults.
        super(config.canID);
        configFactoryDefault();
        setNeutralMode(config.brakeMode);
        configSelectedFeedbackSensor(config.feedbackDevice, config.pidIdx, config.timeout);
        setInverted(config.inverted);
        setSensorPhase(config.sensorPhase);
        setStatusFramePeriod(config.statusFramePeriod, config.timeout);
        configNominalOutputForward(config.nominalOutputForward, config.timeout);
        configNominalOutputReverse(config.nominalOutputReverse, config.timeout);
        configPeakOutputForward(config.peakOutputForward, config.timeout);
        configPeakOutputReverse(config.peakOutputReverse, config.timeout);
        selectProfileSlot(config.slotIdx, config.pidIdx);
        config_kP(config.slotIdx, config.P, config.timeout);
        config_kI(config.slotIdx, config.I, config.timeout);
        config_kD(config.slotIdx, config.D, config.timeout);
        config_kF(config.slotIdx, config.F, config.timeout);
        config_IntegralZone(config.slotIdx, config.integralZone, config.timeout);
    }
    
    /* Stop the motor */
    public void stop() {
        this.set(ControlMode.PercentOutput, 0);
    }

    public void runMotor(double speed) {
        this.set(ControlMode.PercentOutput, speed);
    }

}
