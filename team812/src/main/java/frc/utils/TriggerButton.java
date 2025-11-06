// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TriggerButton extends Trigger {

  public TriggerButton(XboxController controller, XboxController.Axis axis) {
    super(() -> controller.getRawAxis(axis.value) >= 0.5);
  }

}
