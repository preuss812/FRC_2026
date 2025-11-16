// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.GotoPoseCommand;
import frc.robot.commands.PreussSwerveControllerCommand;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;

/** 
 * This class supplies data to create trajectories from any point on the field to the Processor 
 * The hope is that these will be useful for autonomous mode and possibly (but unlikey) for enable semi-automatic driving during teleop.
 * The strategy is the divide the field into approximately 2 meter squares and then create plans based on the
 * best path to travel from each starting square.
 */
public class TrajectoryPlans {

    private static boolean debug = true;
    public static int numXSquares = 8;
    public static int numYSquares = 4;
    public static double dx = FieldConstants.xMax/numXSquares;
    public static double dy = FieldConstants.yMax/numYSquares;
    
    public static int AUTO_MODE_ROBOT_DECIDES;
    public static int AUTO_MODE_MOVE_OFF_LINE_AND_STOP;
    public static int AUTO_MODE_DO_NOTHING;
    public static int AUTO_MODE_MY_BARGE_TO_OPPOSITE;
    public static int AUTO_MODE_MY_BARGE_TO_FAR_SIDE;
    public static int AUTO_MODE_MY_BARGE_TO_NEAR_SIDE;
    public static int AUTO_MODE_CENTER_STRAIGHT;
    public static int AUTO_MODE_THEIR_BARGE_TO_NEAR_SIDE;
    public static int AUTO_MODE_THEIR_BARGE_TO_FAR_SIDE;
    public static int AUTO_MODE_MY_BARGE_TO_CENTER;

    // For now, default speeds are coming from Constants.AutoConstants which has the slow/precision speeds.
    // the 0.9 is to allow headroom for the robot to make sure it does not run out of time during the FollowTrajectoryCommand.
    public static final TrajectoryConfig m_forwardTrajectoryConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond*0.9,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared*0.9*4.0)
        .setKinematics(DriveConstants.kDriveKinematics);

    public static final TrajectoryConfig m_reverseTrajectoryConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond*1.0,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared*1.0*4.0)
        .setKinematics(DriveConstants.kDriveKinematics)
        .setReversed(true);
    
    // Having 2 configs was causing confusion so now there is just one.
    /*public static final TrajectoryConfig m_fullSpeedTrajectoryConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics)
        .setReversed(true);
        ;
    public static final TrajectoryConfig m_defaultTrajectoryConfig = m_debugTrajectoryConfig;
    */

    // Define a gridded map of the field to define a path from each square to the blue alliance processor.
    // Currently these paths are crude and need some refinement if they are to be used during a match.
    public static final Pose2d[][] blueProcessorWaypoints = new Pose2d[][]
    {
        // column 0
        {
            fieldSquareToPose(1,0),
            fieldSquareToPose(2,0),
            fieldSquareToPose(1,1),
            fieldSquareToPose(0,2)
        },
        // column 1
        {
            fieldSquareToPose(1,0),
            fieldSquareToPose(2,0),
            fieldSquareToPose(0,2),
            fieldSquareToPose(0,2),
        },
        // column 2
        {
            fieldSquareToPose(2,0),
            fieldSquareToPose(2,1),
            fieldSquareToPose(3,2),
            fieldSquareToPose(3,2)
        },
        // column 3
        {
            fieldSquareToPose(3,0),
            fieldSquareToPose(3,1),
            fieldSquareToPose(3,1),
            fieldSquareToPose(3,2)
        },
        // column 4
        {
            fieldSquareToPose(4,1),
            fieldSquareToPose(4,2),
            fieldSquareToPose(3,2),
            fieldSquareToPose(3,3)
        },
        // column 5
        {
            fieldSquareToPose(4,1),
            fieldSquareToPose(4,2),
            fieldSquareToPose(4,2),
            fieldSquareToPose(3,2)
        },
        // column 6
        {
            fieldSquareToPose(5,0),
            fieldSquareToPose(7,2),
            fieldSquareToPose(5,2),
            fieldSquareToPose(5,3)
        },  
        // column 7
        {
            fieldSquareToPose(7,1),
            fieldSquareToPose(7,2),
            fieldSquareToPose(6,3),
            fieldSquareToPose(6,3)
        }   
    };

    // Create a red alliance version of the plans by transforming the blue alliance plans
    public static final Pose2d[][] redProcessorWaypoints = transformPoses(blueProcessorWaypoints, FieldConstants.blueToRedTransform);

    /**
     * Construct a Trajectory plans object.  Currently this does nothing as the whole class is static.
    */
    public TrajectoryPlans() {
    }

    /**
     * Convert an array of Pose2d objects (=a plan) using the supplied transform
     * @param trajectoryPlan - The Pose2d objects to be transformed.
     * @param transformPose - The Transform2d to be used.
     * @return Pose2d[][] containing the transformed Pose2d objects.
     */
    public static Pose2d[][] transformPoses(Pose2d[][] trajectoryPlan, Pose2d transformPose) {
        Pose2d[][] newPlan = new Pose2d[numXSquares][numYSquares];
        for (int i = 0; i < numXSquares; i++) {
            for (int j = 0; j < numYSquares; j++) {
                Pose2d sourcePose = trajectoryPlan[numXSquares-i-1][numYSquares-j-1];
                Pose2d transformedPose = sourcePose.relativeTo(transformPose);
                newPlan[i][j] = transformedPose;
            }
        }
        return newPlan;
    }
    
    /**
     * Create a list of poses from the startingPose to the end of the path using the trajectory plan grid as a map.
     * @param trajectoryPlan - the map from each field square to some destination (e.g. the Processsor)
     * @param startingPose - a location on the field used as a starting point for the plan.
     * @return - a list of Pose2d object to get from the startingPose to the destination.
     */
    public static List<Pose2d> planTrajectory( Pose2d[][] trajectoryPlan, Pose2d startingPose) {
        List<Pose2d> list = new ArrayList<>();
        // convert the coordinates into indexes for 2 by 2 meter squares.
        // 0,0 is the lower left of the field by the Red Alliance source.
        int[] ij = poseToFieldSquare(startingPose);
        int i = ij[0];
        int j = ij[1];
        int n = 0;
        boolean done = false;
        list.add(startingPose);
        while (!done) {
            Pose2d nextPose = trajectoryPlan[i][j];
            int [] nextid = poseToFieldSquare(nextPose);
            //if we stay in the same square, we are done.
            if (nextid[0] == i && nextid[1] == j) {
                done = true;
            } else {
                list.add(nextPose);
                n++;
                i = nextid[0];
                j = nextid[1];
                //if (n > 8) 
                //    done = true;
            }
        }
        return list;
    }

    /**
     * return the (i,j) grid position on the field from a location on the field.
     * @param fieldLocation - a Pose2d with the field location
     * @return - an int[2] array with the (i,j) field square for that location.
     */
    public static int[] poseToFieldSquare(Pose2d fieldLocation) {
        int[] square = new int[2];
        square[0] = MathUtil.clamp((int)(fieldLocation.getX()/dx), 0, numXSquares-1);
        square[1] = MathUtil.clamp((int)(fieldLocation.getY()/dy), 0, numYSquares-1);
        return square;
    }
    
    /**
     * return a Pose2d object for a field square representing where in that square the robot should drive.
     * @param i - (int) the i-axis location of the field square.
     * @param j - (int) the j-axis location of the field square.
     * @return - a Pose2d for driving to/through that field square.  Currently the center of that square.
     */
    public static Pose2d fieldSquareToPose(int i, int j) {
        double x = dx*(i+0.5);
        double y = dy*(j+0.5);

        if (i == 0) {
            x += 0.3*dx;
        } else if (i == 1) {
            x -= 0.4*dx;
        } else if (i == 3) {
            x -= 0.3*dx;
        }
        if (j == 1) {
            y -= 0.4 *dy;
        } else if (j == 2) {
            y += 0.2 * dy;
        } else if (j == 3) {
            y -= 0.1*dy;
        }
        
        return new Pose2d(x,y, new Rotation2d(0));
        
    }

    /**
     * create a Trajectory using the driveTrain and config using the list of waypoints.
     * @param waypoints - an array of Pose2d objects describing the starting location, waypoints and ending location for the driving.
     * @param config - a TrajectoryConfig describing the driving parameters (eg max speed, acceleration, etc).
     * @return - a Trajectory object with smoothed (spline) path for the driving path or null if no path could be calculated.
     */
    public static Trajectory createTrajectory(Pose2d[] waypoints, TrajectoryConfig config) {
        Trajectory trajectory = null;
        Pose2d startingPose = waypoints[0];
        Pose2d endingPose = waypoints[waypoints.length - 1];
        boolean useListOfPoses = true;

        try {

            if (useListOfPoses) {
                // Try the version of generateTrajectory that takes a list of poses instead of a list of translations.
                // The hope is that this generates better robot rotational control - dph 2025-01-28
                List<Pose2d> waypointList = Arrays.asList(waypoints);
                trajectory = TrajectoryGenerator.generateTrajectory( waypointList, config != null ? config : m_reverseTrajectoryConfig);
            } else {
                // Generate a trajectory with a starting and ending pose and a list of translation waypoints.
                List<Translation2d>  translationWaypoints = new ArrayList<Translation2d>();
                //Pose2d fakeTargetPose = new Pose2d(endingPose.getTranslation(), new Rotation2d(0));
                for (int i = 1; i < waypoints.length - 1; i++) {
                translationWaypoints.add(waypoints[i].getTranslation());
                }
                trajectory = TrajectoryGenerator.generateTrajectory(          
                    startingPose,  // We are starting where we are.
                    // Pass through these zero interior waypoints, this should probably be something to make sure we dont crash into other robots.
                    translationWaypoints,
                    endingPose,
                    config != null ? config : m_reverseTrajectoryConfig
                ); // use default config is none was specified.
            }
            //if (debug)
                //RobotContainer.m_PoseEstimatorSubsystem.field2d.getObject("trajectory").setTrajectory(trajectory);
        }
        catch (Exception e) {
            // Let null return as the trajectory and the caller must handle it.
        }
        return trajectory;
    }
    
    
    /**
     * Pose facing reef, create a pose that rotates the robot so the camera faces the center of the reef.
     * @param x - a double that is the x field coordinate.
     * @param y = a double that is the y field coordinate.
     * @return - a Pose2d object using the supplied x and y and a rotation that will rotate the robot to face the reef.
     */
    public static Pose2d poseWithCameraFacingTheReef(boolean convertToRed, double x, double y) {
        if (convertToRed) {
            return new Pose2d(x, y, new Rotation2d(FieldConstants.robotHeadingForCameraToRedReefCenter(x,y)));
        } else {
            return new Pose2d(x, y, new Rotation2d(FieldConstants.robotHeadingForCameraToBlueReefCenter(x,y)));
        }
    }
    
    /**
     * Returns a rotation for the robot to face the reef.  Used as a lambda function to supply the rotation to the SwerveControllerCommand.
     * @param poseEstimatorSubsystem
     * @return - the rotation in radians for the robot to face the reef in field coordinates.
     */
    public static Rotation2d reefFacingRotationSupplier(PoseEstimatorSubsystem poseEstimatorSubsystem) {
        Rotation2d rotation = new Rotation2d(
                Autonomous.robotHeadingForCameraToReefCenter(
                    poseEstimatorSubsystem.getCurrentPose().getTranslation()
                ) 
        );
        return rotation;
    } 

    
    /**
     * redTrajectory - create a trajectory for the red alliance from blue alliance waypoints
     * @param - Pose2d[] the blue alliance waypoints
     * @param - the trajectory config
     * @return - the trajectory through the waypoints
     */
    public static Trajectory createRedTrajectory(
          Pose2d[] blueWaypoints
        , TrajectoryConfig config
    ) {
        Pose2d[] redWaypoints = new Pose2d[blueWaypoints.length];
        for (int i = 0; i < blueWaypoints.length; i++) {
            redWaypoints[i] = FieldConstants.BlueToRedPose(blueWaypoints[i]);
        }
        Trajectory trajectory = createTrajectory(redWaypoints, config);
        return trajectory;
    }

    /**
     * This method checks our assumptions that the field is (almost) perfectly rotated around the center of the field.
     * Since we are transforming blue trajectories and locations to make red trajectories and locations, we need
     * the transformed coordinates to be correct.
     * @return - true if the assumptions are met, otherwise false.
     */
    public static boolean checkAprilTagMirroring(PoseEstimatorSubsystem poseEstimatorSubsystem) {
        if (!debug) return true;    // Dont waste time if we are not debugging.
        boolean result = true; // Assume success.  Will reset if there is a mismatch.
        double translationThreshold = 0.002; // 2mm
        double rotationThreshold = 0.001; // 0.001 Radians which is about 1/20th of a degree
        for (int i = 1; i <= 22; i++) {
            if (!Utilities.comparePoses(
                poseEstimatorSubsystem.getAprilTagPose(i),
                poseEstimatorSubsystem.getAprilTagPose(FieldConstants.complementaryAprilTag[i]),
                translationThreshold,
                rotationThreshold
                )
            ) {
                result = false;
            }
        }
        
        return result;
    }

    /**
     * Check that all the trajectory paths generated do not cause a crash.
     * This may throw a null pointer exception.
     */
    public static void checkAllTrajectories() throws NullPointerException {
        if (!debug) return;  // Dont waste time if we are not debugging.
        
        // Verify that the apriltag info matches our expectations
        SmartDashboard.putBoolean("AprilMirrorCheck", checkAprilTagMirroring(RobotContainer.m_PoseEstimatorSubsystem));
        
        // Try all possible plans to makesure there are now obvious bad moves in the plans.
        for (int i = 1; i < 8; i++) {
            for (int j = 1; j < 4; j++) {
            SmartDashboard.putNumber("I", i);
            SmartDashboard.putNumber("J", j);
            
            List<Pose2d> blueProcessorPlan = TrajectoryPlans.planTrajectory(
                TrajectoryPlans.blueProcessorWaypoints, TrajectoryPlans.fieldSquareToPose(i, j)
            );
            }
        }
        // Try all possible plans to makesure there are now obvious bad moves in the plans.
        for (int i = 1; i < 8; i+= 1) {
            for (int j = 1; j < 4; j+= 1) {
            SmartDashboard.putNumber("I", i);
            SmartDashboard.putNumber("J", j);
            List<Pose2d> redProcessorPlan = TrajectoryPlans.planTrajectory(
                TrajectoryPlans.redProcessorWaypoints, TrajectoryPlans.fieldSquareToPose(i, j)
            );
            }
        }
    }

    public static List<Pose2d> sanitizeWaypoints(List<Pose2d> waypoints) {
        // set up the rotation for each waypoint to point the robot to the next waypoint.
        List<Pose2d>  sanitizedWaypoints = new ArrayList<Pose2d>();

        for (int i = 0; i < waypoints.size(); i++) {
            Pose2d pose = waypoints.get(i);
    
            if (i == 0) {
            if (waypoints.size() > 1) {
                Pose2d thisPose = waypoints.get(i);
                Pose2d nextPose = waypoints.get(i+1);
                double directionToNext = Math.atan2(nextPose.getTranslation().getY() - thisPose.getTranslation().getY(), nextPose.getTranslation().getX() - thisPose.getTranslation().getX());  
                sanitizedWaypoints.add(new Pose2d(waypoints.get(i).getTranslation(), new Rotation2d(directionToNext))); // Lying about the orientation to get smoother path.
            } else {
                sanitizedWaypoints.add(pose); // There is only one pose, should not happen but keep the orientation.
            }
            } else if (i == waypoints.size() -1) {
            sanitizedWaypoints.add(pose); // For the ending pose, keep the requested ending orientation.
            } else {
            // Not the end, point to the next waypoint.
            Pose2d lastPose = waypoints.get(i-1);
            Pose2d thisPose = waypoints.get(i);
            Pose2d nextPose = waypoints.get(i+1);
            double directionFromLast = Math.atan2(thisPose.getTranslation().getY() - lastPose.getTranslation().getY(), thisPose.getTranslation().getX() - lastPose.getTranslation().getX());  
            double directionToNext = Math.atan2(nextPose.getTranslation().getY() - thisPose.getTranslation().getY(), nextPose.getTranslation().getX() - thisPose.getTranslation().getX());  
            double direction = (MathUtil.angleModulus(directionFromLast) + MathUtil.angleModulus(directionToNext))/2.0;
            direction = MathUtil.angleModulus(directionToNext);
            sanitizedWaypoints.add(new Pose2d(waypoints.get(i).getTranslation(), new Rotation2d(direction)));
            }
        }
        return sanitizedWaypoints;
    }
}
