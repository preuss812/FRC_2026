// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


/** Add your docs here. */
public class PreussSparkMax extends SparkMax {
    // Next 2 ara probably not needed
    private RelativeEncoder enc = this.getEncoder();
    private SparkClosedLoopController closedLoopController;

    //SparkPIDController pid = this.getPIDController();
    public PreussSparkMax(PreussMotorConfig preussConfig) {

        super(preussConfig.canID, preussConfig.sparkMotorType);
        enc = this.getEncoder();
        closedLoopController = getClosedLoopController();

        SparkMaxConfig motorConfig = new SparkMaxConfig();
        // This sets up 1:1 info which is probably wrong.
        motorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

        
        super.set(0.0);
        /*
        * Configure the closed loop controller. We want to make sure we set the
        * feedback sensor as the primary encoder.
        */
        motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control. We don't need to pass a closed
            // loop slot, as it will default to slot 0.
            .p(preussConfig.P)
            .i(preussConfig.I)
            .d(preussConfig.D)
            .outputRange(preussConfig.peakOutputReverse, preussConfig.peakOutputForward)
            // Set PID values for velocity control in slot 1 -- probably not needed
            .p(preussConfig.P, ClosedLoopSlot.kSlot1)
            .i(preussConfig.I, ClosedLoopSlot.kSlot1)
            .d(preussConfig.D, ClosedLoopSlot.kSlot1)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
            .feedForward
            // kV is now in Volts, so we multiply by the nominal voltage (12V)
            .kV(12.0 / 5676, ClosedLoopSlot.kSlot1);

        motorConfig.closedLoop.maxMotion
            // Set MAXMotion parameters for position control. We don't need to pass
            // a closed loop slot, as it will default to slot 0.
            .cruiseVelocity(1000)
            .maxAcceleration(1000)
            .allowedProfileError(1)
            // Set MAXMotion parameters for velocity control in slot 1
            .maxAcceleration(500, ClosedLoopSlot.kSlot1)
            .cruiseVelocity(6000, ClosedLoopSlot.kSlot1)
            .allowedProfileError(1, ClosedLoopSlot.kSlot1);

        if (preussConfig.brakeMode == com.ctre.phoenix.motorcontrol.NeutralMode.Brake) {
            motorConfig.idleMode(IdleMode.kBrake);
        } else {
            motorConfig.idleMode(IdleMode.kCoast);
        }
        motorConfig.inverted(preussConfig.inverted);
        
        /*
        * Apply the configuration to the SPARK MAX.
        *
        * kResetSafeParameters is used to get the SPARK MAX to a known state. This
        * is useful in case the SPARK MAX is replaced.
        *
        * kPersistParameters is used to ensure the configuration is not lost when
        * the SPARK MAX loses power. This is useful for power cycles that may occur
        * mid-operation.
        */
        configure(motorConfig, com.revrobotics.ResetMode.kResetSafeParameters,  com.revrobotics.PersistMode.kNoPersistParameters);

    }

    // This is from a code example and seems unnecessary
    @Override
    public void set(double speed) {
        // System.out.println("SparkMax " + getDeviceId() + " set to " + speed);
        super.set(speed);
    }

}
