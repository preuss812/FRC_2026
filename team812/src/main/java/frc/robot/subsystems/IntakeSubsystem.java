// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    // Shooter settings
    private final SparkMax motor;
    private final SparkMaxConfig motorConfig;
    private final SparkClosedLoopController closedLoopController;    

    /* State variables related to addressing intake jams
     */
    private int highCurrentCounter = 0;
    private int cycleCounter = 0;
    private int oscillationCounter =0;
    private boolean isOscillating = false;
    private double targetOutput = 0.0; // exposes the set point to the class
    

  public IntakeSubsystem(int intakeCANId) {
    
    motor = new SparkMax(intakeCANId, MotorType.kBrushless);
    closedLoopController = motor.getClosedLoopController();

    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    motorConfig = new SparkMaxConfig();
    motorConfig.smartCurrentLimit(IntakeConstants.currentLimit);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    SmartDashboard.putBoolean("Intake", false);
  }

    /*
     * 2026-03-16 dano added these states to periodic to handle jams
     * States: Normal, High Current in Normal, High Current Warning, Oscilation, Jammed
     * State transition is dependent upon this
     * data: motor output current, high current timer, isOscillating, oscillationCounter
     */
    private void oscillateIntake() {
    	cycleCounter++;
	    if( cycleCounter < 10 ) {
	      closedLoopController.setSetpoint(targetOutput, ControlType.kDutyCycle);
	} else if( oscillationCounter < 20) { // maybe twice the counter?
	      closedLoopController.setSetpoint(0, ControlType.kDutyCycle);
	} else {
	    cycleCounter = 0;
	    oscillationCounter++;
	}
    }
    private void resetOscillation(boolean setPointZero) {
	    isOscillating = false;
	    oscillationCounter =0;
	    cycleCounter = 0;
    	highCurrentCounter = 0;
      if( setPointZero ) {
        targetOutput = 0;
	      closedLoopController.setSetpoint(targetOutput, ControlType.kDutyCycle);
      }
    }
    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      double currentAmps = motor.getOutputCurrent();
      SmartDashboard.putNumber("Intake Motor Current", currentAmps);
      /*
      SmartDashboard.putBoolean("Intake Motor Jammed", !isOscillating);

      if( currentAmps > 70.0 && 
          targetOutput > 0.1 &&
          !isOscillating ) {
	       highCurrentCounter++;
      } else if ( !isOscillating ) {
	       highCurrentCounter = 0; // reset if > 70 didn't last for the number of cycles listed below
      }
      // Periodic runs 20ms, therefore 100 counts is 2 seconds
      if( highCurrentCounter > 100 &&
	        !isOscillating ) {
	        isOscillating = true;
	        oscillationCounter = 0;
	        cycleCounter = 0;
	        highCurrentCounter = 0;
      }
      if( isOscillating ) {
        if( oscillationCounter > 10 ) {
          //closedLoopController.setSetpoint(0, ControlType.kDutyCycle);
          resetOscillation(true);
        } else {
	        oscillateIntake();
        }
        
        if( currentAmps < 70.0 &&
	          cycleCounter < 10 ) {
	            resetOscillation(false);
            }
      } else {
	      closedLoopController.setSetpoint(targetOutput, ControlType.kDutyCycle);
      }
      */
  }

  public void runMotor(double pOut){
    SmartDashboard.putBoolean("Intake", true);
    targetOutput = MathUtil.clamp(pOut, -1, 1);
    /*
    if( targetOutput <= 0.01 ) {
      resetOscillation(false);
    }
    */

    closedLoopController.setSetpoint(targetOutput, ControlType.kDutyCycle);
  }

  
  public void stop() {
    targetOutput = 0.0;
    // resetOscillation(false);
    closedLoopController.setSetpoint(targetOutput, ControlType.kDutyCycle);
    SmartDashboard.putBoolean("Intake", false);
  }

}
