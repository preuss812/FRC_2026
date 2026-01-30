// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BlackBoxSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FlywheelTest extends Command {
  private final FlywheelSubsystem motor;
  private final  BlackBoxSubsystem blackBox;
  /** Creates a new MotorTest. */
  public FlywheelTest(FlywheelSubsystem motor, BlackBoxSubsystem blackBox) {
    this.motor = motor;
    this.blackBox = blackBox;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(motor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double knobPos = blackBox.getPotValue(0);
    SmartDashboard.putNumber("Black Box Rotation", knobPos);
    motor.runMotor(knobPos);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
