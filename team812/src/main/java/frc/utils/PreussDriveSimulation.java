package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PoseEstimatorSubsystem;

/**
 * This class is useful for autonomous or semi autonomous commands
 * It accepts x,y,theta speeds and updates the poseEstimator so that the motion can
 * be viewed in the shuffleboard or other visualization tools.
 * Note that the simulated motion does not take into account physics but instead 
 * simply updates the robot position based on the commanded speeds.
 */
public class PreussDriveSimulation {

    private  PoseEstimatorSubsystem poseEstimatorSubsystem = RobotContainer.m_poseEstimatorSubsystem;
    private  Pose2d simulatedRobotPose;


    public PreussDriveSimulation(PoseEstimatorSubsystem poseEstimatorSubsystem){
        
        this.poseEstimatorSubsystem = poseEstimatorSubsystem;
        this.reset();
    }

    public void reset() {
        simulatedRobotPose = poseEstimatorSubsystem.getCurrentPose();
    }

    /*
    * Drive method for simulation mode.
    * @param xSpeed - speed in the x direction (meters per second)
    * @param ySpeed - speed in the y direction (meters per second)
    * @param rotationSpeed - rotational speed (radians per second)
    */
    public void drive(double xSpeed, double ySpeed, double rotationSpeed) {
        if (RobotContainer.isSimulation()) {
            // For debug, update the forced robot location based on the x,y,theta applied.
            double seconds=0.020; // Rate that the simulation applies the changes to the robot's position.
            simulatedRobotPose = poseEstimatorSubsystem.getCurrentPose();
            simulatedRobotPose = new Pose2d(
              simulatedRobotPose.getX()+xSpeed*seconds
            , simulatedRobotPose.getY()+ySpeed*seconds
            , simulatedRobotPose.getRotation().plus(new Rotation2d(rotationSpeed*seconds)));

            poseEstimatorSubsystem.setCurrentPose(simulatedRobotPose);

            // Update the DriveSubsystemSRX pose in case we are intentionally driving with using the camera.
            RobotContainer.m_robotDrive.resetOdometry(simulatedRobotPose);

            /*
             * TODO:implement simulation of the gyro angle.
             * I wanted to update the drivetrain pose and the gyro angle for a more complete simulation
             * but that functionality is not currently implemented.
             * As a consequence, reliance on the gyro angle direclly will not work as the gyro angle will likely always be zero.
             */
            //RobotContainer.m_robotDrive.setAngleDegrees(simulatedRobotPose.getRotation().getDegrees());
            //RobotContainer.m_robotDrive.m_gyro.getAngle();
        }
    }

    public void setCurrentPose(Pose2d pose) {
        simulatedRobotPose = pose;
        poseEstimatorSubsystem.setCurrentPose(simulatedRobotPose);
    }

    public Pose2d getCurrentPose() {
        return poseEstimatorSubsystem.getCurrentPose();
    }
}
