// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.Utilities;
import frc.robot.subsystems.PoseEstimatorSubsystem;

/** Add your docs here. */
public class PoseEstimatorCamera extends PhotonCamera {
    private final Transform3d cameraToRobotTransform;
    //private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
    private PoseEstimatorSubsystem poseEstimatorSubsystem;
    private double previousPipelineTimestamp = 0;
    private int m_lastAprilTagSeen = VisionConstants.NO_TAG_FOUND;
    private static final PhotonPipelineResult m_emptyPipeline = new PhotonPipelineResult();
    private PhotonPipelineResult lastPipeLineResult = m_emptyPipeline;
    private final boolean debug = false;

    public PoseEstimatorCamera(String name, Transform3d cameraToRobotTransform) {
        super(name);
        this.cameraToRobotTransform = cameraToRobotTransform;
    }

    public VisionResult getNewVisionMeasurement() {
        VisionResult visionMeasurement = null;
        var pipelineResult = getResult();
        var resultTimestamp = pipelineResult.getTimestampSeconds();
        if ((resultTimestamp > previousPipelineTimestamp) && pipelineResult.hasTargets()) {
            previousPipelineTimestamp = resultTimestamp;

            var targets = pipelineResult.getTargets();
      
            //var target = pipelineResult.getBestTarget();
            var target = getBestTarget(targets);
            if (target != null) {
                var fiducialId = target.getFiducialId();
                // Get the tag pose from field layout - consider that the layout will be null if it failed to load
                Optional<Pose3d> tagPose = poseEstimatorSubsystem.getAprilTagPose3d(fiducialId);
                // If we have a field location for this tag, use it to update the robot's position.
                if (tagPose.isPresent()) {
                    var targetPose = tagPose.get();
                    Transform3d camToTarget = target.getBestCameraToTarget();                Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

                    visionMeasurement = new VisionResult(camPose.transformBy(cameraToRobotTransform), fiducialId, resultTimestamp);
                }
            }
        }
        return visionMeasurement;
    }

    
    public PhotonPipelineResult getResult() {
        PhotonPipelineResult result = lastPipeLineResult;

        // Grab the latest result. We don't care about the timestamp from NT - the metadata header has
        // this, latency compensated by the Time Sync Client
        var results = getAllUnreadResults();
        if (debug) SmartDashboard.putNumber("PV size", results.size());
        // This is different than in 2024.
        for (PhotonPipelineResult pipelineResult : results) {
          if (pipelineResult.hasTargets()) {
            //var target = pipelineResult.getBestTarget();
            var target = getBestTarget((pipelineResult.getTargets()));
            if (target != null) {
              var bestFiducialId = target.getFiducialId();
              SmartDashboard.putNumber("PV ambiguity", target.getPoseAmbiguity());

              if (target.getPoseAmbiguity() <= VisionConstants.maximumAmbiguity 
              && bestFiducialId >= VisionConstants.MIN_FIDUCIAL_ID 
              && bestFiducialId <= VisionConstants.MAX_FIDUCIAL_ID) {
                result = pipelineResult;
                m_lastAprilTagSeen = bestFiducialId;
                lastPipeLineResult = pipelineResult;
              } else {
                m_lastAprilTagSeen = VisionConstants.NO_TAG_FOUND;
              }
            } else {
              m_lastAprilTagSeen = VisionConstants.NO_TAG_FOUND;
            }
          } else {
            m_lastAprilTagSeen = VisionConstants.NO_TAG_FOUND;
          }
        }
        if (debug) SmartDashboard.putNumber("PV last", m_lastAprilTagSeen);
        return result;
    }
    /**
     * find the best april tag for our purposes.
     * For 2025 that is the april tag with acceptable ambiguity 
     * having the closest to the center of the field of view
     * else largest area.
     * @param targets - the list of targets seen.  Typically there will be only 1.
     * @return - the best target.
     */
    public PhotonTrackedTarget getBestTarget(List<PhotonTrackedTarget> targets) {
      PhotonTrackedTarget result = null;
      double bestYaw = Math.PI; // Sentinel value. cannt have a greater heading than +/- field of view which is 100 degrees.
      double bestArea = 0.0;    // Smallest possible area.

      String s = "";

      //double bestAmbiguity = 1; // Sentinel value. cannot be more than 100% ambiguous.
      int targetsSeen = 0;
      for (PhotonTrackedTarget t : targets) {
        if (t.getPoseAmbiguity() <= VisionConstants.maximumAmbiguity
        && t.getFiducialId() >= VisionConstants.MIN_FIDUCIAL_ID 
        && t.getFiducialId() <= VisionConstants.MAX_FIDUCIAL_ID ) {
          targetsSeen++;
          if (targetsSeen == 1) {
            // First seen, therefore it must be the best so far :-)
            bestYaw = Math.abs(t.getYaw());
            bestArea = t.getArea();
            //bestAmbiguity = t.getPoseAmbiguity();
            result = t;
            s += t.getFiducialId()+":"+t.getYaw()+"   ";
          /* } else if (t.getPoseAmbiguity() < bestAmbiguity) {
            // Lower ambiguity so better for calculating location
                bestYaw = t.getYaw();
                bestArea = t.getArea();
                bestAmbiguity = t.getPoseAmbiguity();
                result = t; 
                targetsSeen++;*/
            } else if (Math.abs(t.getYaw()) < bestYaw) {
                // closest to the center of the field of view 
                bestYaw = Math.abs(t.getYaw());
                bestArea = t.getArea();
                //bestAmbiguity = t.getPoseAmbiguity();
                result = t;
                s += t.getFiducialId()+":"+t.getYaw()+"   ";

            } else if (Math.abs(t.getYaw()) == bestYaw && t.getArea() > bestArea) {
                // biggest => closest
                bestYaw = Math.abs(t.getYaw());
                bestArea = t.getArea();
                //bestAmbiguity = t.getPoseAmbiguity();
                result = t;
                s += t.getFiducialId()+":"+t.getYaw()+"   ";

            }
            }
        }
        SmartDashboard.putString("PV s", s);
        return result;
    }

    public void setPoseEstimatorSubsystem(PoseEstimatorSubsystem poseEstimatorSubsystem) {
        this.poseEstimatorSubsystem = poseEstimatorSubsystem;
    }
}
