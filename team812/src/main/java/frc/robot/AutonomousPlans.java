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
import frc.robot.subsystems.AllianceConfigurationSubsystem;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;

/** 
 * This class manages the available autonomous plans for the robot.
 */
public class AutonomousPlans {
    private static boolean debug = true;
   
    public static ArrayList<Trajectory> autoPaths = new ArrayList<Trajectory>();
    public static ArrayList<String>     autoNames = new ArrayList<String>();
    public static ArrayList<Pose2d[]>   waypoints = new ArrayList<Pose2d[]>();
    public static ArrayList<Integer>    expectedAprilTags = new ArrayList<Integer>();
    public static ArrayList<Trajectory> trajectories = new ArrayList<Trajectory>();
    public static ArrayList<Pose2d>     finalPoses = new ArrayList<Pose2d>();
    public static ArrayList<Pose2d>     startingPoses = new ArrayList<Pose2d>();
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

    
    /**
     * 
     * @param name       - the display name for this autonomous mode.
     * @param waypoints  - an array of poses to be used to generate the driving path
     * @param config     - the trajectory configuration to be used to generate the driving path.
     * @param aprilTagID - the apriltag expected to be seen when the driving begins.
     *                     This is used to verify the starting position and possibly terminate the
     *                     autonomous mode if the robot is not where it is expected to be.
     *                     For now, the checking is done in the VerifyStartingPositionCommand which
     *                     is added to a sequential command group which would prevent this command from
     *                     being executed.
     * 
     * Note: the grouping of commands here and in Autonomous.java is not ideal and should be refactored.
     */
    public static void addAutoMode(
        String name,
        Pose2d[] waypoints,
        Pose2d finalPose,
        TrajectoryConfig config,
        int aprilTagID,
        Alliance alliance) {

        autoNames.add(name);
        Robot.autoChooser.addOption(autoNames.get(autoNames.size()-1), autoNames.size()-1);
        if (Alliance.Blue == alliance) {
            startingPoses.add(waypoints[0]);
            trajectories.add(TrajectoryPlans.createTrajectory(waypoints, config));
            finalPoses.add(finalPose);
            expectedAprilTags.add(aprilTagID);
        } else  {
            startingPoses.add(FieldConstants.BlueToRedPose(waypoints[0]));
            trajectories.add(TrajectoryPlans.createRedTrajectory(waypoints, config));
            finalPoses.add(FieldConstants.BlueToRedPose(finalPose));
            expectedAprilTags.add(Constants.FieldConstants.complementaryAprilTag[aprilTagID]);
        }
    }

    /**
     * Create predefined autonomous routines for use during the autonomous period.
     * For now, 6 routines are defined.  One for each april tag on the reef.
     * It creates paths for either blue and red alliances depending on the argument.
     * Data is entered in blue alliance and transformed to red if we are in the red alliance.
     * 
     * Note: Path numbering is problematic so be careful when adding new options to keep the
     * numbering consistent.
     * 
     */
    public static void buildAutoPlans(Alliance alliance) {
        // In case we get called multiple times, clear out the arraylists.
        // Just for debugging and will not happen in a match.
        autoNames.clear();
        waypoints.clear();
        expectedAprilTags.clear();
        trajectories.clear();
        finalPoses.clear();
        startingPoses.clear();
    
        // The staring poses will be on the blue starting line facing back toward the blue drive station
        Rotation2d startingRotation = new Rotation2d(0.0);
        double offsetFromAprilTag = Units.inchesToMeters(40); // 0.5 meters from the april tag
        double offsetToTouchReef = Units.inchesToMeters(0.0); // -5 inches meters from the april tag Note: this should be 0.0 so something is off somewhere.
        // Get/create poses for each Reef April tag and barge april tag
        // for omre concise coding below.
        Pose2d AT14 = RobotContainer.m_PoseEstimatorSubsystem.getAprilTagPose(14);
        Pose2d AT15 = RobotContainer.m_PoseEstimatorSubsystem.getAprilTagPose(15);
        Pose2d AT17 = RobotContainer.m_PoseEstimatorSubsystem.getAprilTagPose(17);
        Pose2d AT18 = RobotContainer.m_PoseEstimatorSubsystem.getAprilTagPose(18);
        Pose2d AT19 = RobotContainer.m_PoseEstimatorSubsystem.getAprilTagPose(19);
        Pose2d AT20 = RobotContainer.m_PoseEstimatorSubsystem.getAprilTagPose(20);
        Pose2d AT21 = RobotContainer.m_PoseEstimatorSubsystem.getAprilTagPose(21);
        Pose2d AT22 = RobotContainer.m_PoseEstimatorSubsystem.getAprilTagPose(22);

        Pose2d nearAT17 = DriveConstants.robotRearAtPose(AT17, offsetFromAprilTag);
        Pose2d nearAT18 = DriveConstants.robotRearAtPose(AT18, offsetFromAprilTag);
        Pose2d nearAT19 = DriveConstants.robotRearAtPose(AT19, offsetFromAprilTag);
        Pose2d nearAT20 = DriveConstants.robotRearAtPose(AT20, offsetFromAprilTag);
        Pose2d nearAT21 = DriveConstants.robotRearAtPose(AT21, offsetFromAprilTag);
        Pose2d nearAT22 = DriveConstants.robotRearAtPose(AT22, offsetFromAprilTag);
        Pose2d atAT17 = DriveConstants.robotRearAtPose(AT17, offsetToTouchReef);
        Pose2d atAT18 = DriveConstants.robotRearAtPose(AT18, offsetToTouchReef);
        Pose2d atAT19 = DriveConstants.robotRearAtPose(AT19, offsetToTouchReef);
        Pose2d atAT20 = DriveConstants.robotRearAtPose(AT20, offsetToTouchReef);
        Pose2d atAT21 = DriveConstants.robotRearAtPose(AT21, offsetToTouchReef);
        Pose2d atAT22 = DriveConstants.robotRearAtPose(AT22, offsetToTouchReef);
        //TrajectoryConfig config = m_debugTrajectoryConfig;
        TrajectoryConfig config = TrajectoryPlans.m_reverseTrajectoryConfig;

        // Add the defualt plan which is not yet defined, for now do nothing.
        AUTO_MODE_ROBOT_DECIDES = autoNames.size();
        autoNames.add("Robot Makes the Plan");
        Robot.autoChooser.addOption(autoNames.get(autoNames.size()-1), autoNames.size()-1);
        waypoints.add(new Pose2d[]{}); // Empty array.
        trajectories.add(new Trajectory());
        finalPoses.add(new Pose2d(AllianceConfigurationSubsystem.getStartLine() - 1.0, FieldConstants.yCenter, startingRotation.plus(new Rotation2d(Math.PI/2.0))));
        startingPoses.add(new Pose2d(AllianceConfigurationSubsystem.getStartLine(), FieldConstants.yCenter, startingRotation.plus(new Rotation2d(Math.PI/2.0))));
        expectedAprilTags.add(0);

        // Add the defualt plan which is not yet defined, for now do nothing.
        AUTO_MODE_MOVE_OFF_LINE_AND_STOP = autoNames.size();
        autoNames.add("Drive off Line and Stop");
        Robot.autoChooser.addOption(autoNames.get(autoNames.size()-1), autoNames.size()-1);
        waypoints.add(new Pose2d[]{}); // Empty array.
        trajectories.add(new Trajectory());
        finalPoses.add(new Pose2d(FieldConstants.blueStartLine - 1.0, FieldConstants.yCenter, startingRotation));
        startingPoses.add(new Pose2d(FieldConstants.blueStartLine, FieldConstants.yCenter, startingRotation));
        expectedAprilTags.add(0);

        // Add the defualt plan which is not yet defined, for now do nothing.
        AUTO_MODE_DO_NOTHING = autoNames.size();
        autoNames.add("Do Nothing");
        Robot.autoChooser.addOption(autoNames.get(autoNames.size()-1), autoNames.size()-1);
        waypoints.add(new Pose2d[]{}); // Empty array.
        trajectories.add(new Trajectory());
        finalPoses.add(new Pose2d(FieldConstants.blueStartLine, FieldConstants.yCenter, startingRotation));
        startingPoses.add(new Pose2d(FieldConstants.blueStartLine, FieldConstants.yCenter, startingRotation));
        expectedAprilTags.add(0);

        // Build a path adding it to the autoChooser which will select the autonomous routine
        AUTO_MODE_MY_BARGE_TO_OPPOSITE = autoNames.size();
        addAutoMode(
            "My Barge to Opposite"
            ,new Pose2d[] {
                new Pose2d(FieldConstants.blueStartLine,AT14.getY(), startingRotation),
                //new Pose2d(AT20.getX(),AT14.getY(), startingRotation),
                //new Pose2d(AT19.getX(),AT14.getY(), startingRotation),
                new Pose2d(FieldConstants.zeroToReef,AT14.getY(), startingRotation),
                new Pose2d(FieldConstants.zeroToReef * 0.66 - 1,AT18.getY()+1.0, new Rotation2d(Math.PI*0.5)),
                nearAT18
            },
            atAT18,
            config,
            20,
            alliance);

        // Build a path adding it to the autoChooser which will select the autonomous routine
        AUTO_MODE_MY_BARGE_TO_FAR_SIDE = autoNames.size();
        addAutoMode("My Barge to Far Side"
            ,new Pose2d[] {
                new Pose2d(FieldConstants.blueStartLine,AT14.getY(), startingRotation),
                new Pose2d(AT19.getX()-0.5, AT14.getY()-0.2, startingRotation.plus(new Rotation2d(Units.degreesToRadians(30)))),
                //new Pose2d(AT19.getX()-0.6, AT14.getY()-0.2, AT19.getRotation()),
                //new Pose2d(AT19.getX(),AT14.getY(), startingRotation),
                //nearAT19           
            }
            ,atAT19
            ,config
            ,20
            ,alliance
        );

        AUTO_MODE_MY_BARGE_TO_NEAR_SIDE = autoNames.size();
        addAutoMode(
            "My Barge to Near Side"
            , new Pose2d[] {
                new Pose2d(FieldConstants.blueStartLine,AT14.getY(), startingRotation)
                //new Pose2d(AT21.getX()+0.5,AT14.getY(), startingRotation.plus(new Rotation2d(Units.degreesToRadians(0)))),
                //poseWithCameraFacingTheReef((AT20.getX()+FieldConstants.blueStartLine)/2.0,AT14.getY()),  Adds extra squiggles to the path
                //nearAT20
            }
            ,atAT20
            , config
            , 20
            , alliance
        );
        
        // Build a path adding it to the autoChooser which will select the autonomous routine
        AUTO_MODE_CENTER_STRAIGHT = autoNames.size();
        addAutoMode(
            "Center Straight"
            , new Pose2d[] {
                new Pose2d(FieldConstants.blueStartLine ,FieldConstants.yCenter, startingRotation),
                new Pose2d((AT21.getX()+FieldConstants.blueStartLine)/2.0,FieldConstants.yCenter, startingRotation),
                //nearAT21
            }
            ,atAT21
            , config
            , 21
            , alliance
            );

        // Build a path adding it to the autoChooser which will select the autonomous routine
        AUTO_MODE_THEIR_BARGE_TO_NEAR_SIDE = autoNames.size();
        addAutoMode(
            "Their Barge to Near Side"  
            , new Pose2d[] {
                new Pose2d(FieldConstants.blueStartLine,AT15.getY(), startingRotation),
                new Pose2d(AT21.getX()+0.5,AT15.getY(), startingRotation),
            nearAT22
            }
            ,atAT22
            , config
            , 22
            , alliance
        );

        // Build a path adding it to the autoChooser which will select the autonomous routine
        AUTO_MODE_THEIR_BARGE_TO_FAR_SIDE = autoNames.size();
        addAutoMode("Their Barge to Far Side"
            , new Pose2d[] {
                new Pose2d(FieldConstants.blueStartLine,AT15.getY(), startingRotation),
                new Pose2d(AT22.getX()-1.1,AT15.getY()+0.2, startingRotation.plus(new Rotation2d(Units.degreesToRadians(-30)))),               
            }
            ,atAT17
            , config
            , 22
            , alliance
        );

        // Build a path adding it to the autoChooser which will select the autonomous routine
        AUTO_MODE_MY_BARGE_TO_CENTER = autoNames.size();
        addAutoMode("Right 45 Degrees"
            , new Pose2d[] {
                TrajectoryPlans.poseWithCameraFacingTheReef(false, FieldConstants.blueStartLine, AT14.getY()),
                //poseWithCameraFacingTheReef(FieldConstants.blueStartLine-0.2, FieldConstants.yCenter + 1),
                nearAT21
            }
            ,atAT21
            , config
            , 20
            , alliance
        );

        SmartDashboard.putData("AutoSelector", Robot.autoChooser);
    }
}
