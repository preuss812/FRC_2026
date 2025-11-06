// Code from frc-7028-2023/src/main/java/frc/robot/subsystems/PoseEstimatorSubsystem.java
// Copied Jan 22, 2024 as a seed for our Pose Estimator.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilities;
import frc.utils.PoseEstimatorCamera;
import frc.utils.VisionResult;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;

public class PoseEstimatorSubsystem extends SubsystemBase {

  private final PoseEstimatorCamera[] cameras;
  private final DriveSubsystemSRX drivetrainSubsystem;
  
  private final AprilTagFieldLayout aprilTagFieldLayout;
  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others. 
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.

  /**
   * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
   */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  
  // Uncomment the next line to support AdvantageScope
  //private static StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic("/MyPose", Pose2d.struct).publish();

  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

  private final SwerveDrivePoseEstimator poseEstimator;

  public final Field2d field2d = new Field2d();
  private int m_lastAprilTagSeen = VisionConstants.NO_TAG_FOUND;
  private boolean debug = true;

  public PoseEstimatorSubsystem(PoseEstimatorCamera[] cameras, DriveSubsystemSRX drivetrainSubsystem) {
    
    this.cameras = cameras;

    // There is a circular relationship between PoseEsitmatorCamera and PoseEstimatorSubsystem.
    // This hack seeks to minimize that problem for now.
    for (PoseEstimatorCamera camera : cameras) {
      camera.setPoseEstimatorSubsystem(this);
    }
    
    this.drivetrainSubsystem = drivetrainSubsystem;
    AprilTagFieldLayout layout;
    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeAndyMark.m_resourceFile);
      // We are always using the blue alliance as the origin of the field.
      layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    } catch(IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      layout = null;
    }
    this.aprilTagFieldLayout = layout;

    poseEstimator =  new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        drivetrainSubsystem.getRotation(),
        drivetrainSubsystem.getModulePositions(),
        new Pose2d(),
        stateStdDevs,
        visionMeasurementStdDevs);
    
    SmartDashboard.putData("Field", field2d);
   
  }

  @Override
  public void periodic() {
    Utilities.toSmartDashboard("PE CurrentPose", getCurrentPose());

    // Check each camera for new vision measurements.
    int fiducialId = VisionConstants.NO_TAG_FOUND;
    for (PoseEstimatorCamera camera : cameras) {
      VisionResult visionMeasurement = camera.getNewVisionMeasurement();
      // Dont use the lifecam in 2025 if the arm is not in a good position.
      // This has not been tested.  The camera is on the arm so it's moving as the arm moves.
      /*
      if (camera.getName().equals("Microsoft_LifeCam_HD-3000") && 
         (RobotContainer.m_ShoulderRotationSubsystem.getCurrentPosition() > ShoulderConstants.kShoulderScoreAlgaeInProcessorPosition + 5.0
          || RobotContainer.m_ElbowRotationSubsystem.getCurrentPosition() < ElbowConstants.kElbowScoreAlgaeInProcessorPosition + 0.5)) {
        continue;
      }
      */
      if (visionMeasurement != null) {
        poseEstimator.addVisionMeasurement(visionMeasurement.pose().toPose2d(), visionMeasurement.timestamp());
        fiducialId = visionMeasurement.fiducialId();
      }
    }
    m_lastAprilTagSeen = fiducialId;

    // Update pose estimator with drivetrain sensors
    poseEstimator.update(
      drivetrainSubsystem.getRotation(),
      drivetrainSubsystem.getModulePositions());

    // Update the Shuffleboard with the robot's position on the field
    field2d.setRobotPose(getCurrentPose());
    //publisher.set(getCurrentPose()); 
    if (debug) {
      SmartDashboard.putNumber("Pose X", getCurrentPose().getX());
      SmartDashboard.putNumber("Pose Y", getCurrentPose().getY());
    }
  }

  public String getFomattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f) %.2f degrees", 
        pose.getX(), 
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    if (newPose != null) {
      poseEstimator.resetPosition(
      drivetrainSubsystem.getRotation(),
      drivetrainSubsystem.getModulePositions(),
      newPose);
    }
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

  public Pose2d getAprilTagPose(int aprilTagId) {
    Optional<Pose3d> pose3d;
    Pose2d pose2d = new Pose2d();
    pose3d = aprilTagFieldLayout.getTagPose(aprilTagId);
    if (pose3d.isPresent())
      pose2d = pose3d.get().toPose2d();
    return pose2d;
  }

  public Optional<Pose3d> getAprilTagPose3d(int fiducialId) {
    Optional<Pose3d> tagPose = aprilTagFieldLayout == null ? Optional.empty() : aprilTagFieldLayout.getTagPose(fiducialId);
    return tagPose;
  }

  // For autonomous to prevent motion if the robot cannot see an apriltag and therefore is in an unknown location.
  public BooleanSupplier tagInViewSupplier = () -> (m_lastAprilTagSeen != VisionConstants.NO_TAG_FOUND);
  public boolean tagInView () {
    return (m_lastAprilTagSeen != VisionConstants.NO_TAG_FOUND);
  }

  public int lastAprilTagSeen() {
    return m_lastAprilTagSeen;
  }
  // Return the pose for a robot to be directly in front of the specified apriltag
  public Pose2d robotFrontAtApriltag(int id, double offset) {
    return DriveConstants.robotFrontAtPose(getAprilTagPose(id), offset);
  }

  // Return the pose for a robot to be directly in front of the specified apriltag
  public Pose2d robotRearAtApriltag(int id, double offset) {
    return DriveConstants.robotRearAtPose(getAprilTagPose(id), offset);
  }  
  
  
}
