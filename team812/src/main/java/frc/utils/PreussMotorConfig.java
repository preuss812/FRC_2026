package frc.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * Data structure used to configure a PreussMotor.
 * Currently supports only WPI_TalonSRX which is the ususal motor controller at Preuss
 */
public class PreussMotorConfig {
    public int canID = 0;
    public NeutralMode brakeMode = NeutralMode.Brake;
    public ControlMode controlMode = ControlMode.PercentOutput;
    public boolean inverted = false;   // if true, motor output is inverted.
    public FeedbackDevice feedbackDevice = FeedbackDevice.QuadEncoder;
    public boolean sensorPhase = true;  // if true, sensor phase is positive.
    public int statusFramePeriod = 10; // milliseconds.
    public int timeout = 10; // milliseconds.
    public double nominalOutputForward = 0.0;
    public double nominalOutputReverse = 0.0;
    public double peakOutputForward = 0.8;  // 80%
    public double peakOutputReverse = -0.8;  // -80%
    public double P = 0.0;
    public double I = 0.0;
    public double D = 0.0;
    public double F = 0.0;
    public double integralZone = 0.0; // window used to prevent integral windup.
    public int slotIdx = 0;
    public int pidIdx = 0;
    public double cruiseVelocity = 150.0;
    public double acceleration = 150.0;
    public LimitSwitchNormal forwardLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
    public LimitSwitchNormal reverseLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
    public boolean clearEncoderOnForwardLimit = false; // if true, encoder will be zeroed when forward limit switch is hit.
    public boolean clearEncoderOnReverseLimit = false; // if true, encoder will be zeroed when reverse limit switch is hit.
    public int magicMotionFramePeriod = 10; // milliseconds.
    public int pidFramePeriod = 10; // milliseconds.

    public PreussMotorConfig(int canID) {
        this.canID = canID;
    }

    public int getCanID() {
        return canID;
    }

    public PreussMotorConfig setCanID(int canID) {
        this.canID = canID;
        return this;
    }

    public NeutralMode getBrakeMode() {
        return brakeMode;
    }

    public PreussMotorConfig setBrakeMode(NeutralMode brakeMode) {
        this.brakeMode = brakeMode;
        return this;
    }

    public ControlMode getControlMode() {
        return controlMode;
    }

    public PreussMotorConfig setControlMode(ControlMode controlMode) {
        this.controlMode = controlMode;
        return this;
    }

    public boolean isInverted() {
        return inverted;
    }

    public PreussMotorConfig setInverted(boolean inverted) {
        this.inverted = inverted;
        return this;
    }

    public FeedbackDevice getFeedbackDevice() {
        return feedbackDevice;
    }

    public PreussMotorConfig setFeedbackDevice(FeedbackDevice feedbackDevice) {
        this.feedbackDevice = feedbackDevice;
        return this;
    }

    public boolean isSensorPhase() {
        return sensorPhase;
    }

    public PreussMotorConfig setSensorPhase(boolean sensorPhase) {
        this.sensorPhase = sensorPhase;
        return this;
    }

    public int getStatusFramePeriod() {
        return statusFramePeriod;
    }

    public PreussMotorConfig setStatusFramePeriod(int statusFramePeriod) {
        this.statusFramePeriod = statusFramePeriod;
        return this;
    }

    public int getTimeout() {
        return timeout;
    }

    public PreussMotorConfig setTimeout(int timeout) {
        this.timeout = timeout;
        return this;
    }

    public double getNominalOutputForward() {
        return nominalOutputForward;
    }

    public PreussMotorConfig setNominalOutputForward(double nominalOutputForward) {
        this.nominalOutputForward = nominalOutputForward;
        return this;
    }

    public double getNominalOutputReverse() {
        return nominalOutputReverse;
    }

    public PreussMotorConfig setNominalOutputReverse(double nominalOutputReverse) {
        this.nominalOutputReverse = nominalOutputReverse;
        return this;
    }

    public double getPeakOutputForward() {
        return peakOutputForward;
    }

    public PreussMotorConfig setPeakOutputForward(double peakOutputForward) {
        this.peakOutputForward = peakOutputForward;
        return this;
    }

    public double getPeakOutputReverse() {
        return peakOutputReverse;
    }

    public PreussMotorConfig setPeakOutputReverse(double peakOutputReverse) {
        this.peakOutputReverse = peakOutputReverse;
        return this;
    }

    public double getP() {
        return P;
    }

    public PreussMotorConfig setP(double p) {
        P = p;
        return this;
    }

    public double getI() {
        return I;
    }

    public PreussMotorConfig setI(double i) {
        I = i;
        return this;
    }

    public double getD() {
        return D;
    }

    public PreussMotorConfig setD(double d) {
        D = d;
        return this;
    }

    public double getF() {
        return F;
    }

    public PreussMotorConfig setF(double f) {
        F = f;
        return this;
    }

    public double getIntegralZone() {
        return integralZone;
    }

    public PreussMotorConfig setIntegralZone(double integralZone) {
        this.integralZone = integralZone;
        return this;
    }

    public int getSlotIdx() {
        return slotIdx;
    }

    public PreussMotorConfig setSlotIdx(int slotIdx) {
        this.slotIdx = slotIdx;
        return this;
    }

    public int getPidIdx() {
        return pidIdx;
    }

    public PreussMotorConfig setPidIdx(int pidIdx) {
        this.pidIdx = pidIdx;
        return this;
    }

    public double getCruiseVelocity() {
        return cruiseVelocity;
    }

    public PreussMotorConfig setCruiseVelocity(double cruiseVelocity) {
        this.cruiseVelocity = cruiseVelocity;
        return this;
    }

    public double getAcceleration() {
        return acceleration;
    }

    public PreussMotorConfig setAcceleration(double acceleration) {
        this.acceleration = acceleration;
        return this;
    }

    public LimitSwitchNormal getForwardLimitSwitchNormal() {
        return forwardLimitSwitchNormal;
    }

    public PreussMotorConfig setForwardLimitSwitchNormal(LimitSwitchNormal forwardLimitSwitchNormal) {
        this.forwardLimitSwitchNormal = forwardLimitSwitchNormal;
        return this;
    }

    public LimitSwitchNormal getReverseLimitSwitchNormal() {
        return reverseLimitSwitchNormal;
    }

    public PreussMotorConfig setReverseLimitSwitchNormal(LimitSwitchNormal reverseLimitSwitchNormal) {
        this.reverseLimitSwitchNormal = reverseLimitSwitchNormal;
        return this;
    }

    public boolean isClearEncoderOnForwardLimit() {
        return clearEncoderOnForwardLimit;
    }

    public PreussMotorConfig setClearEncoderOnForwardLimit(boolean clearEncoderOnForwardLimit) {
        this.clearEncoderOnForwardLimit = clearEncoderOnForwardLimit;
        return this;
    }

    public boolean isClearEncoderOnReverseLimit() {
        return clearEncoderOnReverseLimit;
    }

    public PreussMotorConfig setClearEncoderOnReverseLimit(boolean clearEncoderOnReverseLimit) {
        this.clearEncoderOnReverseLimit = clearEncoderOnReverseLimit;
        return this;
    }

    public int getMagicMotionFramePeriod() {
        return magicMotionFramePeriod;
    }

    public PreussMotorConfig setMagicMotionFramePeriod(int magicMotionFramePeriod) {
        this.magicMotionFramePeriod = magicMotionFramePeriod;
        return this;
    }

    public int getPidFramePeriod() {
        return pidFramePeriod;
    }

    public PreussMotorConfig setPidFramePeriod(int pidFramePeriod) {
        this.pidFramePeriod = pidFramePeriod;
        return this;
    }

}
