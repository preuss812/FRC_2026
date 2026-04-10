/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class CANConstants {
        public static final int kSwerveRightRearRotate = 23;
        public static final int kSwerveRightRearDrive = 24;
        public static final int kSwerveRightRearCANCoder = 34;

        public static final int kSwerveRightFrontRotate = 21;
        public static final int kSwerveRightFrontDrive = 22;
        public static final int kSwerveRightFrontCANCoder = 32;

        public static final int kSwerveLeftRearRotate = 25;
        public static final int kSwerveLeftRearDrive = 26;
        public static final int kSwerveLeftRearCANCoder = 36;

        public static final int kSwerveLeftFrontRotate = 27;
        public static final int kSwerveLeftFrontDrive = 28;
        public static final int kSwerveLeftFrontCANCoder = 38;

        //public static final int kShooterMotor2 = 40;// unused

        //public static final int kElbowMotor2 = 44;
        public static final int kIntakeMotor = 43;
        public static final int kFollowerMotor = 44; // -1 => there is no follower motor. 44 is the assigned value for the follower motor if we had one.
        public static final int kShooterMotor = 45;
        public static final int kFeederMotor = 46;
        public static final int kIndexerMotor = 47;
        //public static final int kShooterElevationMotor = 41;
        public static final int kIntakeDeploymentMotor = 48;
        public static final int kBellyMotor1 = 50;
        public static final int kBellyMotor2 = 51;

        public static final int kPDP = 0; // was 50 until 3/12/2024
        //public static final int kPCM = 51;
    }

    
    public static final class OIConstants {
        public static final int kLeftJoystick = 0;
        public static final int kRightJoystick = 1;
        public static final int kControlBox = 2;
        

        // Controlbox interfaces
        public static final int kControlBoxPotX = 0;
        public static final int kControlBoxPotY = 1;
        public static final int[] kControlBox3WaySwitch = {1,2}; // 3 position switch, see truth table below
        public static final int kControlBoxTottleButton = 3; // normally closed (1), pressed is open (0)
        public static final int kControlBoxSw1 = 4; // up is (0), down is (1) for all two position switches
        public static final int kControlBoxSw2 = 5;
        public static final int kControlBoxSw3 = 6;
        public static final int kControlBoxSw4 = 7;

        /* 
        kControlBox3WaySwitch implements the following states
           sw1   |   sw2   | 3-way position
        ---------+---------+---------------------
            0    |    0    | Left
            0    |    1    | Center
            1    |    0    | No such position
            1    |    1    | Right
        */

        // Xbox joystick constants
        public static final int kXboxAButton = 1;
        public static final int kXboxBButton = 2;
        public static final int kXboxXButton = 3;
        public static final int kXboxYButton = 4;
        public static final int kXboxLBumper = 5;
        public static final int kXboxRBumper = 6;
        public static final int kXboxSelect = 7;
        public static final int kXboxStart = 8;
        public static final int kXboxLTrigger = 9;
        public static final int kXboxRTrigger = 10;

        public static final int kDriverControllerPort = 2;
        public static final double kDriveDeadband = 0.10;
    }

    /**
     * Define analog ports used on the RoboRio
     */
    public static final class AnalogIOConstants {
        /*
        * no pneumatics this year:
        public   static final int kPressureTransducer = 0;
        public static final int kPressureOffset = -20;
        public static final int kPressureRange = 200;
        */
        public static final int kShooterElevationEncoder = 0;
    }

    public static final class PidConstants {
        public static final double kProportionalDriveStraight = 0.05;
     
        public static final double kShooterElevation_kP = 0.2;
        public static final double kShooterElevation_kI = 0.0005;
        public static final double kShooterElevation_IntegralZone=15;
        public static final double kShooterElevation_kD = 0.0;
        public static final double kShooterElevation_kF = 0.0;
        /*
        public static final double kArmExtension_kP = 0.3; //3.0;
        public static final double kArmExtension_kI = 0.0;
        public static final double kArmExtension_kD = 0.0;
        public static final double kArmExtension_kF = 0.0;
        public static final double kArmExtension_rampRate = 0.5;
        public static final double kPorportionalBalanceForward = 0.05;
        public static final double kProportionalBalanceBackward = 0.05;
        */
        
        
        /*
        public static final double kShooter_kP = 2.7; // TODO are these needed and tuned?
        public static final double kShooter_kI = 0.0;
        public static final double kShooter_kD = 0.0;
        public static final double kShooter_kF = 0.0;
        */
    }
    
    // Define locations on the field that may be useful for semi-automatic driving.
    public static final class FieldConstants {
        // All units in Meters.
        // These values are derived from april tag locations and field drawings in the game manual or cad drawings.
        public static final double fieldLength = Units.inchesToMeters(651.22); // This is X
        public static final double fieldWidth = Units.inchesToMeters(317.69); // This is Y

        // Handy values
        public static final double xMin = 0.00;
        public static final double xMax = fieldLength; // Meters 
        public static final double yMin = 0.00;
        public static final double yMax = fieldWidth; // Meters
        public static final double xCenter = (xMax - xMin)/2.0;
        public static final double yCenter = (yMax - yMin)/2.0;
        public static final double xSquareSize = xMax/8;
        public static final double ySquareSize = yMax/4;
        public static final double blueHubCenterX = Units.inchesToMeters(182.11);
        public static final Translation2d blueHubCenter = new Translation2d(blueHubCenterX, yCenter);
        public static final Translation2d redHubCenter = BlueToRedTranslation(blueHubCenter);
        public static final double redHubCenterX = redHubCenter.getX();
        public static final double hubToStart = Units.inchesToMeters(88.0);
        
        // Handy X coordinates:
        public static final double blueStartLine =  Units.inchesToMeters(156.61);
        public static final double redStartLine = xMax - blueStartLine;   

        // Helper functions to translate Blue coordinates to Red coordinates.
        // This is most likely to be used for creating red alliance autonomous routines
        // from blue alliance autonomous routines.  For 2025 this is a rotation about the center
        // of the field of play.  In 2024 this was a mirroring of the field about the center of the field.

        /**
         * Transform a Translation2d for the blue alliance to the complementary Translation2d for the red alliance.
         * @param blueRotation - the Translation2d for the robot in the blue alliance.
         * @return - the Translation2d for the robot in the red alliance.
         */
        public static Translation2d BlueToRedTranslation(Translation2d blueTranslation) {
            return new Translation2d(xMax - blueTranslation.getX(), yMax - blueTranslation.getY());
        }

        /**
         * Transform a Rotation2d for the blue alliance to the complementary Rotation2d for the red alliance.
         * @param blueRotation - the Rotation2d for the robot in the blue alliance.
         * @return - the Rotation2d for the robot in the red alliance.
         */
        public static Rotation2d BlueToRedRotation(Rotation2d blueRotation) {
            return blueRotation.rotateBy(new Rotation2d(Math.PI));
        }

        /**
         * Transform a Pose2d for the blue alliance to the complementary Pose2d for the red alliance.
         * @param bluePose - The Pose2d for the robot in the blue alliance.
         * @return         - The Pose2d for the robot in the red alliance.
         */
        public static Pose2d BlueToRedPose(Pose2d bluePose) {
            return new Pose2d(BlueToRedTranslation(bluePose.getTranslation()), BlueToRedRotation(bluePose.getRotation()));
        }

        // Field Coordinate transformations for alliances.
        public static int BlueAlliance = 0;
        public static int RedAlliance = 1;
        public static Pose2d blueToRedTransform = new Pose2d(xMax, yMax, new Rotation2d(Math.PI));

        public static int[] complementaryAprilTag = new int[] {
            0, // Artifact of zero indexing.
            17, // 1
            18, // 2
            19, // 3
            20, // 4
            21,
            22,
            23,
            24,
            25,
            26,
            27,
            28,  
            29,
            30,
            31,
            32,
            1,
            2,
            3,
            4,
            5,
            6,  
            7,
            8,
            9,
            10,
            11,
            12,
            13,
            14,
            15,
            16 
        };

        
    }
    
    public static final class BrakeConstants {
        public static final String kNotBraking = "NotBraking";
        public static final String kUnknown    = "Unknown";
        public static final String kBraking    = "Braking";
    }

    public static final class VisionConstants {
        /**
        * Physical location of the camera on the robot, relative to the center of the robot.
        * Distance in meters, angles in radians
        */

        public static final int NO_TAG_FOUND = -1;
        public static final int MIN_FIDUCIAL_ID = 1;
        public static final int MAX_FIDUCIAL_ID = 32;
        public static final double maximumAmbiguity = 0.2;
        

        public static double rearCameraXOffsetToRobot = Units.inchesToMeters(-12.0);
        public static double rearCameraYOffsetToRobot = Units.inchesToMeters(8.75);
        public static double rearCameraHeightToGround = Units.inchesToMeters(13.5);
        public static double rearCameraRoll  = Units.degreesToRadians(0.0);
        public static double rearCameraPitch = Units.degreesToRadians(0.0); 
        public static double rearCameraYaw   = Units.degreesToRadians(180.0); // Rear facing camera.
        public static final double rearCameraHeading = Units.degreesToRadians(180.0); // Rear facing camera
        public static final Transform3d REAR_APRIL_CAMERA_TO_ROBOT = new Transform3d(
            new Translation3d(rearCameraXOffsetToRobot,rearCameraYOffsetToRobot,rearCameraHeightToGround),
            new Rotation3d(rearCameraRoll, rearCameraPitch, rearCameraYaw)
        );
        public static final Transform3d ROBOT_TO_REAR_APRIL_CAMERA = REAR_APRIL_CAMERA_TO_ROBOT.inverse();


        public static double aprilCameraXOffsetToRobot = Units.inchesToMeters(-3.0);
        public static double aprilCameraYOffsetToRobot = Units.inchesToMeters(9.0);
        public static double aprilCameraHeightToGround = Units.inchesToMeters(20.5);
        public static double aprilCameraRoll  = Units.degreesToRadians(0.0);
        public static double aprilCameraPitch = Units.degreesToRadians(0.0); 
        public static double aprilCameraYaw   = Units.degreesToRadians(0.0); // Front facing camera.
        public static final double aprilCameraHeading = Units.degreesToRadians(0); // Front facing camera
        public static final Transform3d APRIL_CAMERA_TO_ROBOT = new Transform3d(
            new Translation3d(aprilCameraXOffsetToRobot,aprilCameraYOffsetToRobot,aprilCameraHeightToGround),
            new Rotation3d(aprilCameraRoll, aprilCameraPitch, aprilCameraYaw)
        );
        public static final Transform3d ROBOT_TO_APRIL_CAMERA = APRIL_CAMERA_TO_ROBOT.inverse();


        public enum AprilTag {
            UNKNOWN(0),
            RED_OUTPOST(13),
            RED_TOWER(15),
            BLUE_OUTPOST(29),
            BLUE_TOWER(31);
            
            
            private int id;
            private AprilTag(int id) {
                this.id = id;
            }
            public int id() {return id; }

        }
    
    }

    public static final class RotationConstants {
        public static final Rotation2d zero = new Rotation2d(0);
        public static final Rotation2d rotate90 = new Rotation2d(Math.PI/2.0);
        public static final Rotation2d rotate180 = new Rotation2d(Math.PI);
        public static final Rotation2d rotate270 = new Rotation2d(Math.PI*3.0/2.0);

    }

    // The constants in DriveConstants and ModuleConstants are from 
    // https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/subsystems/MAXSwerveModule.java
    // Which is where I sourced the MAXSwerve template - dph - 2024-01-12
    public static final class DriveConstants {
        public static final double kMaxSpeedMetersPerSecond = 4.5; //4.5; 1.0 in the lab // Limit how violently swerve works
        public static final double kMaxAngularSpeed = 3 * Math.PI; // radians per second
    
        public static final double kDirectionSlewRate = 1.2; // radians per second
        public static final double kMagnitudeIncreaseSlewRate = 0.75; // percent per second (1 = 100%)
        public static final double kMagnitudeDecreaseSlewRate = 1.8; // percent per second (1 = 100%)
        public static final double kRotationalIncreaseSlewRate = 1.0;// 2.0; // percent per second (1 = 100%) // UNDO
        public static final double kRotationalDecreaseSlewRate = 2.0;

        // Driving Parameters for PRECISION DrivingMode.  This is a slower mode for more precise positioning.
        public static final double kMaxSpeedMetersPerSecondPM = 1.6; //4.5; 1.0 in the lab // Limit how violently swerve works
        public static final double kMaxAngularSpeedPM = 1 * Math.PI; // radians per second
    
        public static final double kDirectionSlewRatePM = 0.6; // radians per second
        public static final double kMagnitudeIncreaseSlewRatePM = 0.45; // percent per second (1 = 100%)
        public static final double kMagnitudeDecreaseSlewRatePM = 3.6; // percent per second (1 = 100%)
        public static final double kRotationalIncreaseSlewRatePM = 0.3; // percent per second (1 = 100%) // UNDO
        public static final double kRotationalDecreaseSlewRatePM = 4.0;
        
        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(20.25); 
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(20.25);
        // Distance between front and back wheels on robot

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset =0;// -Math.PI / 4;
        public static final double kFrontRightChassisAngularOffset = 0;//Math.PI / 4;
        public static final double kBackLeftChassisAngularOffset = 0;//5 * Math.PI / 4;
        public static final double kBackRightChassisAngularOffset =0;// 3 * Math.PI / 4;
    
        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = CANConstants.kSwerveLeftFrontDrive;
        public static final int kRearLeftDrivingCanId = CANConstants.kSwerveLeftRearDrive;
        public static final int kFrontRightDrivingCanId = CANConstants.kSwerveRightFrontDrive;
        public static final int kRearRightDrivingCanId = CANConstants.kSwerveRightRearDrive;
    
        public static final int kFrontLeftTurningCanId = CANConstants.kSwerveLeftFrontRotate;
        public static final int kRearLeftTurningCanId = CANConstants.kSwerveLeftRearRotate;
        public static final int kFrontRightTurningCanId = CANConstants.kSwerveRightFrontRotate;
        public static final int kRearRightTurningCanId = CANConstants.kSwerveRightRearRotate;

        public static final int kFrontLeftTurningEncoderCanId = CANConstants.kSwerveLeftFrontCANCoder;
        public static final int kRearLeftTurningEncoderCanId = CANConstants.kSwerveLeftRearCANCoder;
        public static final int kFrontRightTurningEncoderCanId = CANConstants.kSwerveRightFrontCANCoder;
        public static final int kRearRightTurningEncoderCanId = CANConstants.kSwerveRightRearCANCoder;

        public static final boolean kGyroReversed = false;

        public static final double kBackToCenterDistance = Units.inchesToMeters(17.5); //was 15.0 until 3/5/2024
        public static final double kFrontToCenterDistance = Units.inchesToMeters(17.5); //was 15.0 until 3/5/2024
        public static final double kBumperWidth = 0; //TODO Restore Units.inchesToMeters(4.25);
        public static final double kRobotWidth = Units.inchesToMeters(27.18)+kBumperWidth*2.0; // Frame width plus 2 bumpers.
        public static final double kRobotLength = Units.inchesToMeters(27.0)+kBumperWidth*2.0; // Frame length plus 2 bumpers.
        public static final double kApproximateStartingY = FieldConstants.yMax - Units.inchesToMeters(36.0); // Meters (ie near the amp)
        public static final double kStartingOrientation = 0.0; // Starting orientation in radians (ie robot back against the alliance wall)
        public static final Translation2d robotCenterToFrontBumper = new Translation2d(kRobotLength/2, 0);
        public static final Translation2d robotCenterToLeftBumper = new Translation2d(0, kRobotWidth/2);
        public static final Translation2d robotCenterToRearBumper = new Translation2d(-(kRobotLength/2), 0);
                public static final Translation2d robotCenterToRightBumper = new Translation2d(0, -kRobotWidth/2);

        
        /**
         * Returns the translation of the robot's center to the center of the front bumper.
         * @param rotation (radians)
         * @return (x,y) (meters) the translation of the robot's center to the center of the front bumper
         */
        public static final Translation2d rotatedRobotFrontBumper(Rotation2d rotation, double offset) {
            return robotCenterToFrontBumper.plus(new Translation2d(offset, 0.0)).rotateBy(rotation);
        }

        /**
         * Returns the translation of the robot's center to the center of the rear bumper.
         * @param rotation (radians)
         * @return (x,y) (meters) the translation of the robot's center to the center of the rear bumper
         */
        public static final Translation2d rotatedRobotRearBumper(Rotation2d rotation, double offset) {
            return robotCenterToRearBumper.minus(new Translation2d(offset, 0)).rotateBy(rotation.plus(RotationConstants.rotate180));
        }

        /**
         * Returns the translation of the robot's center to the center of the left bumper.
         * @param rotation (radians)
         * @return (x,y) (meters) the translation of the robot's center to the center of the left bumper
         */
        public static final Translation2d rotatedRobotLeftBumper(Rotation2d rotation, double offset) {
            return robotCenterToLeftBumper.plus(new Translation2d(offset, 0.0)).rotateBy(rotation.plus(RotationConstants.rotate270));
        }

/**
         * Returns the translation of the robot's center to the center of the right bumper.
         * @param rotation (radians)
         * @return (x,y) (meters) the translation of the robot's center to the center of the right bumper
         */
        public static final Translation2d rotatedRobotRightBumper(Rotation2d rotation, double offset) {
            return robotCenterToRightBumper.plus(new Translation2d(offset, 0.0)).rotateBy(rotation.plus(RotationConstants.rotate90));
        }


        /**
         * Return a pose for the robot to be backing up to the pose with the center of it's rear bumper touching the pose.
         * @param pose - The pose you want to back up to.  Persumably, an AprilTag's pose.
         * @param offset - (meters) the distance to be away from the pose.
         * @return - The pose of the robot backed up to the pose with the center of it's rear bumper aligned to the pose.
         */
        public static Pose2d robotRearAtPose(Pose2d pose, double offset) {
            Translation2d position = pose.getTranslation().plus(rotatedRobotFrontBumper(pose.getRotation(), offset));
            Rotation2d rotation = pose.getRotation(); // .rotateBy(rotate180);
            return new Pose2d(position, rotation);
        }
    
        /**
         * Return a pose for the robot to be facing the pose with the center of it's front bumper touching the pose.
         * @param pose - The pose you want to face. Persumably, an AprilTag's pose.
         * @param offset - (meters) the distance to be away from the pose.
         * @return - The pose of the robot facing the pose with the center of it's front bumper aligned to the pose.
         */
        public static Pose2d robotFrontAtPose(Pose2d pose, double offset) {
            Translation2d position = pose.getTranslation().plus(rotatedRobotFrontBumper(pose.getRotation(), offset));
            Rotation2d rotation = pose.getRotation().rotateBy(RotationConstants.rotate180);
            return new Pose2d(position, rotation);
        }

         /**
         * Return a pose for the robot to be at the pose with the center of it's left bumper touching the pose.
         * @param pose - The pose you want to face. Persumably, an AprilTag's pose.
         * @param offset - (meters) the distance to be away from the pose.
         * @return - The pose of the robot facing the pose with the center of it's left bumper aligned to the pose.
         */
        public static Pose2d robotLeftAtPose(Pose2d pose, double offset) {
            Translation2d position = pose.getTranslation().plus(rotatedRobotLeftBumper(pose.getRotation(), offset));
            Rotation2d rotation = pose.getRotation().rotateBy(RotationConstants.rotate90);  // +90 to face left
            return new Pose2d(position, rotation);
        }

        /**
         * Return a pose for the robot to be at the pose with the center of it's right bumper touching the pose.
         * @param pose - The pose you want to face. Persumably, an AprilTag's pose.
         * @param offset - (meters) the distance to be away from the pose.
         * @return - The pose of the robot facing the pose with the center of it's right bumper aligned to the pose.
         */
        public static Pose2d robotRightAtPose(Pose2d pose, double offset) {
            Translation2d position = pose.getTranslation().plus(rotatedRobotRightBumper(pose.getRotation(), offset));
            Rotation2d rotation = pose.getRotation().rotateBy(RotationConstants.rotate270); // +270 to face right
            return new Pose2d(position, rotation);
        }
    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in a
        // robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 14;
    
        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;
    
        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0); // wheels 4" x 1.5"
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final double kDrivingMotorReduction = 8.14; //(45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
            / kDrivingMotorReduction;
    
        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
            / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
            / kDrivingMotorReduction) / 60.0; // meters per second
    
        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
    
        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians
        public static final double kTurningSRXEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // rotations

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 12.0 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;
    
        public static final double kTurningP = 0.3;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;
    
        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;
    
        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
      }

      public static final class AutoConstants {
        public static final double kAutoSlowdown = 1.0; // 1.0 is full speed, 0.5 is half speed, etc.
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kMaxSpeedMetersPerSecond*kAutoSlowdown;
        public static final double kMaxAccelerationMetersPerSecondSquared = DriveConstants.kMagnitudeIncreaseSlewRate*kAutoSlowdown;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI; //DriveConstants.kMaxAngularSpeed;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI; //2.0; // TODO Undo - DriveConstants.kRotationalIncreaseSlewRate;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
        public static final double kRotationSpeed = -0.05;
        public static final double kShooterTimout = 6.0;
    
        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        public static final int kRobotMakesThePlan = 0;
        public static final int kDriveOffTheLineAndStop = 1;
        public static final int kDoNothing = 2;
      }
    
      public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
      }

    public static final class ColorConstants {
        /* Name: Construction Cone Orange
            URL:  https://www.computerhope.com/cgi-bin/htmlcolor.pl?c=F87431
            RGB:  248, 116, 49
            HSL:  20.20-deg, 93.43%, 58.24%
            HEX: #F87431

            wpilib/util/Color
            https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/util/Color.html
            There are two constructor signatures available, one for
            Integer values and nother for Double values. If you send a
            Double, the values must be adjusted to a 0 to 1 range from
            0 to 255 when passed as integers. Doubles are used so that we 
            can better control precision if needed.
        */

        public static final double kColorConfidenceThreshold = 0.90;
        public static final int    kColorProximityThreshold  = 300; // higher closer, lower is further away
    }

    public static final class UltrasonicConstants {
        public static final int kUltrasonicAnalogPort = 3;
        public static int kPingChannel = 0;
        public static int kEchoChannel = 1;
        public static double kOffsetToBumper = 0.157; // Meters
    }

    public static final class FeederConstants {
        public static final double maxRPM = 6784.0; // Neo Vortex
        public static final double a = ShooterConstants.a;
        public static final double b = ShooterConstants.b;
        public static final double c = ShooterConstants.c;
        public static final double d = ShooterConstants.d;
        public static final double RPMTolerance = 75;
        public static final double RPMToVolts = 442.5;
        public static final double kP = 0.0001;
        public static final double kI = 0.0; // 0.0003/FeederConstants.maxRPM;
        public static final double kD = 0.0;
        public static final double minOutputPercent = -0.8;
        public static final double maxOutputPercent = 0.8;
        public static final double kV = 12.0/maxRPM * 3000.0 / 2900.0; // Docs say 1.0 should be 12.0 but emperically 12.0 is not right.
                public static final boolean inverted = false;
        public static final double shooterFactor = 1; //0.70; // The feeder needs to run slower than the shooter to prevent jamming.  This factor is multiplied by the shooter output to determine the feeder output.
        }

    public static final class IndexerConstants {
        public static final double minOutputPercent = -0.8;
        public static final double maxOutputPercent = 0.8;
        public static final double maxRPM = 6784.0; // Neo Vortex
        public static final double indexerPercentOutput = 0.4;
        public static final int currentLimit = 60;
    }
    
    public static final class ShooterConstants {
        /* - these were from simulation and did not match real world testing.
        public static final double a = 2.61;
        public static final double b = -26.01;
        public static final double c = 388.7;
        public static final double d = 498;
        */
        // Coefficients from Mar 7th emperical results.
        public static final double a = -35.46628;
        public static final double b = 377.17263;
        public static final double c = -883.33910;
        public static final double d = 3063.16870;
        public static final double RPMTolerance = 75;
        public static final double RPMToVolts = 442.5;
        public static final double kP = 0.0006; //0.0006;
        public static final double kI = 0.0; //0.0003/FeederConstants.maxRPM;
        public static final double kD = 0.0;
        public static final double minOutputPercent = -0.8;
        public static final double maxOutputPercent = 0.8;
        public static final double maxRPM = 6784.0; // Neo Vortex
        public static final double kV = 12.0/maxRPM * 3000.0 / 2900.0 * 2600/2555; // Docs say 1.0 should be 12.0 but emperically 12.0 is not right.
        public static final double kS = 0.0;
        public static final double rotationTolerance = Units.degreesToRadians(2.0);
        public static final int currentLimit = 60;
        public enum ShooterMode {IDLE, AUTO_RANGING, UNJAMMING, FIXED_SPEED };
        public static final boolean inverted = true;
        public static final boolean followerInverted = true;

    }

    public static final class IntakeConstants {
        public static final double pickupFuelSpeed = 0.5;
        public static final double ejectFuelSpeed = -0.35;
        public static final int currentLimit = 60;
        public static final double agitateReversedTime = 0.1; // seconds to reverse intake if stuck
        public static final double agitateForwardTime = 0.5; // seconds to run intake forward after reversing
    }

    public static final class IntakeDeploymentConstants {
        public static final double kIntakeDeploymentUpRPM = 1400;
        public static final double kIntakeDeploymentUpSlowlyRPM = 100;
        public static final double kIntakeDeploymentDownRPM = -800;
        public static final double kIntakeDeploymentShakeUpRPM = -800;
        public static final double kIntakeDeploymentShakeDownRPM = 400;
        public static final double kIntakeDeploymentLoweredPosition = -8.0;
        public static final double kP = 0.0002;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double minOutputPercent = -0.80;
        public static final double maxOutputPercent = 0.80;
        public static final double maxRPM = 6784.0; // Neo Vortex
        public static final double kV = 12.0/maxRPM; // Docs say 1.0 should be 12.0 but emperically 12.0 is not right.
        public static final int shakeCycles = 20; // number of periods to move in the same direction before reversing.
        // Constants to convert position in rotations to position in degrees.  These are based on emperical measurements of the mechanism.
        public static final double maxPosition = -11;
        public static final double minPosition = 0.0;
        public static final double minRotation = Units.degreesToRadians(0.0);
        public static final double maxRotation = Units.degreesToRadians(95.0);
        public static final double positionToRotationFactor = (maxRotation - minRotation) / (maxPosition - minPosition);
        public static final double maxFeedForwardPercent = 0.05; // The mximum percent output to use for feed forward.
    }

    public static final class BellyConstants {
        public static final double kP = 0.0001;
        public static final double kI = 0.0; // 0.0003/FeederConstants.maxRPM;
        public static final double kD = 0.0;
        public static final double minOutputPercent = -0.8;
        public static final double maxOutputPercent = 0.8;
        public static final double maxRPM = 6784.0; // Neo Vortex
        public static final double kV = 12.0/maxRPM; // Docs say 1.0 should 
        public static final double feedFuelRPM = -600;
    }
   
    
}

