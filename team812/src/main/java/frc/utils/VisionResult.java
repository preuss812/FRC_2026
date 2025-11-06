// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import edu.wpi.first.math.geometry.Pose3d;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class VisionResult {
  private final Pose3d pose;
  private final int fiducialId;
  private final double timestamp;
  /** Creates a new Pose3dWithTimestamp. */
  public VisionResult(Pose3d pose,int fiducialId, double timestamp) {
    this.pose = pose;
    this.fiducialId = fiducialId;
    this.timestamp = timestamp;
  }

  public Pose3d pose() {
    return this.pose;
  }

  public int fiducialId() {
    return fiducialId;
  }
  public double timestamp() {
    return timestamp;
  }
  
}
