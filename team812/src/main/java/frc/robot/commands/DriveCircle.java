package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.utils.DrivingConfig;

/**
 * This command drives the robot in a circular path.
 * It relies on the GotoPoseCommand to move to successive points along the circle.
 * The results were poor but might improve with different PID tuning or using 
 * code simular to the code used to follow Choreo paths.
 */
public class DriveCircle extends GotoPoseCommand {

  private Integer count;
  private Pose2d circleCenter;
  private Pose2d startingPose;
  private final static Pose2d dummy = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
  private static Integer initcalls = 0;
  private static Integer executecalls = 0;
  private static Integer isFinishedcalls = 0;

  // Define the radius of the circle (in meters)
  private final double radius;

  /** Creates a new DriveCircleCommand. */
  public DriveCircle(DriveSubsystemSRX robotDrive, PoseEstimatorSubsystem poseEstimatorSubsystem, DrivingConfig config, double radius) {
    super(robotDrive, poseEstimatorSubsystem, dummy, false, config); // The relative movePose will be overwritten in the initialize method.
    SmartDashboard.putNumber("inits", initcalls);
    SmartDashboard.putNumber("executes", executecalls);
    SmartDashboard.putNumber("isFinished", isFinishedcalls);
    this.radius = radius;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Get the current pose of the robot as the center of the circle.
    startingPose = RobotContainer.m_PoseEstimatorSubsystem.getCurrentPose();

    // Optional: Offset the center to define where the circle will start (if you want).
    // For example, moving the center by 1 meter on the X-axis:
    circleCenter = new Pose2d(startingPose.getX() - radius, startingPose.getY(), startingPose.getRotation());

    count = 0;
    initcalls++;
    executecalls = 0;
    SmartDashboard.putNumber("inits", initcalls);

    // No need to call super.initialize() because the GotoPoseCommand constructor handles that
  }

  @Override
  public void execute() {
    // Calculate the target position based on a circular trajectory
    double angle = Units.degreesToRadians(count >= 720 ? 0.0 : count%360);  // Convert to radians

    // X and Y follow the parametric equations of a circle: X = r * cos(theta), Y = r * sin(theta)
    double targetX = circleCenter.getX() + radius * Math.cos(angle);
    double targetY = circleCenter.getY() + radius * Math.sin(angle);

    // Update the target pose (we can also add rotation for more realistic motion)
    this.targetPose = new Pose2d(targetX, targetY, startingPose.getRotation());
    onTarget = false;
    super.execute();
    count+= 1;  // Increment the angle

    executecalls++;
    SmartDashboard.putNumber("executes", executecalls);
  }

  @Override
  public boolean isFinished() {
    isFinishedcalls++;
    Pose2d pose = RobotContainer.m_PoseEstimatorSubsystem.getCurrentPose();
    // Finish when the angle has completed a full circle (360 degrees)
    return count >= 720
    && Math.abs(startingPose.getX() - pose.getX()) < Units.inchesToMeters(2.0)
    && Math.abs(startingPose.getY() - pose.getY()) < Units.inchesToMeters(2.0)
    && Math.abs(startingPose.getRotation().getDegrees() - pose.getRotation().getDegrees()) < 2.0;
  }
}
