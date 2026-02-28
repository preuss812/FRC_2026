// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class OldFeederSubsystem extends SubsystemBase {

    // Feeder settings
    private final SparkFlex motor;
    private final SparkFlexConfig motorConfig;
    private final SparkClosedLoopController closedLoopController;    

  public OldFeederSubsystem(int intakeCANId) {
    
    motor = new SparkFlex(intakeCANId, MotorType.kBrushless);
    closedLoopController = motor.getClosedLoopController();


    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    motorConfig = new SparkFlexConfig();
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double pOut = RobotContainer.m_blackBox.getPotValue(1);
    runMotor(pOut);
    SmartDashboard.putNumber("Feeder%Out", pOut);
  }

  public void runMotor(double pOut){
    pOut = MathUtil.clamp(pOut, -1, 1);
    closedLoopController.setSetpoint(pOut, ControlType.kDutyCycle);
  }

  



  public void stop() {
    closedLoopController.setSetpoint(0, ControlType.kDutyCycle);
  }

}