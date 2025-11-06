package frc.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;

/**
 * This class is useful for autonomous or semi autonomous commands
 * It supplies x,y,theta pid controllers and supports viewing the dirving on the shuffleboard
 * in simulation mode for faster debugging.
 * 
 */
public class PreussDriveSimulation {

    private  PoseEstimatorSubsystem poseEstimatorSubsystem = RobotContainer.m_PoseEstimatorSubsystem;
    private  Pose2d simulatedRobotPose;


    public PreussDriveSimulation(PoseEstimatorSubsystem poseEstimatorSubsystem){
        
        this.poseEstimatorSubsystem = poseEstimatorSubsystem;
        this.reset();
    }

    public void reset() {
        simulatedRobotPose = poseEstimatorSubsystem.getCurrentPose();
    }

    public  void drive(double xSpeed, double ySpeed, double rotationSpeed) {
        if (RobotContainer.isSimulation()) {
            // For debug, update the forced robot location based on the x,y,theta applied.
            double seconds=0.020; // Rate that the simulation applies the changes to the robot's position.
            simulatedRobotPose = poseEstimatorSubsystem.getCurrentPose();
            simulatedRobotPose = new Pose2d(
              simulatedRobotPose.getX()+xSpeed*seconds
            , simulatedRobotPose.getY()+ySpeed*seconds
            , simulatedRobotPose.getRotation().plus(new Rotation2d(rotationSpeed*seconds)));

            poseEstimatorSubsystem.setCurrentPose(simulatedRobotPose);
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
