// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.unused;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants.CANConstants;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkLowLevel.MotorType;
//import com.revrobotics.SparkAbsoluteEncoder;
//import com.revrobotics.SparkAbsoluteEncoder.Type;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  // private CANSparkMax lf_rotate;
  // public static SparkAbsoluteEncoder lf_rotate_encoder;

  public DriveTrain() {
    // lf_rotate = new CANSparkMax(CANConstants.kSwerveLeftFrontRotate,MotorType.kBrushless);
    // lf_rotate.set(0.1);
    // lf_rotate_encoder = lf_rotate.getAbsoluteEncoder(Type.kDutyCycle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
