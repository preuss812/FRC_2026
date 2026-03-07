/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.UltrasonicConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DriveChoreoPathCommand;
import frc.robot.commands.DriveCircle;
import frc.robot.commands.DriveCircleThrottle;
import frc.robot.commands.DriveFacingHub;
import frc.robot.commands.DriveRobotCommand;
import frc.robot.commands.DriveWithoutVisionCommand;
import frc.robot.commands.FireAtWillCommand;
import frc.robot.commands.GotoPoseCommand;
import frc.robot.commands.PointCameraTowardHubCommand;
import frc.robot.commands.RandomRobotPositionCommand;
import frc.robot.commands.ResetDriveTrainCommand;
import frc.robot.commands.RotateRobotCommand;
import frc.robot.commands.RotateRobotG2PCommand;
import frc.robot.commands.ShooterTest;
import frc.robot.commands.SimSetRobotPoseCommand;
import frc.robot.subsystems.AllianceConfigurationSubsystem;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.DriveSubsystemSRX.DrivingMode;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeDeploymentSubsystem;
import frc.robot.subsystems.PingResponseUltrasonicSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.utils.PoseEstimatorCamera;
import frc.utils.PreussDriveSimulation;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // private final ExampleCommand m_autoCommand = new
  // ExampleCommand(m_exampleSubsystem);
  //private final DriveTrain m_DriveTrain = new DriveTrain();
  // The robot's subsystems
  public final static DriveSubsystemSRX m_robotDrive = new DriveSubsystemSRX();
  public final SparkFlex m_test = new SparkFlex(50, MotorType.kBrushless);
  public static PoseEstimatorCamera m_atagCamera = new PoseEstimatorCamera("atag812", VisionConstants.ROBOT_TO_APRIL_CAMERA);
  //public static PoseEstimatorCamera m_frontCamera = new PoseEstimatorCamera("Microsoft_LifeCam_HD-3000", VisionConstants.ROBOT_TO_FRONT_CAMERA);

  public static final PoseEstimatorCamera[] cameras = new PoseEstimatorCamera[]{m_atagCamera/*,m_frontCamera*/};
  public static PoseEstimatorSubsystem m_poseEstimatorSubsystem = new PoseEstimatorSubsystem( cameras, m_robotDrive);
  public final static AllianceConfigurationSubsystem m_allianceConfigurationSubsystem = new AllianceConfigurationSubsystem(m_robotDrive, m_poseEstimatorSubsystem);
  private static  boolean isSimulation = !Robot.isReal();
  public static PreussDriveSimulation m_preussDriveSimulation = new PreussDriveSimulation(m_poseEstimatorSubsystem);
  private static boolean debug = true; // To enable debugging in this module, change false to true.
  public static ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem(CANConstants.kShooterMotor1);
  public static IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem(CANConstants.kIntakeMotor);
  public static IntakeDeploymentSubsystem m_IntakeDeploymentSubsystem = new IntakeDeploymentSubsystem(CANConstants.kIntakeDeploymentMotor);
  public static FeederSubsystem m_FeederSubsystem = new FeederSubsystem(CANConstants.kFeederMotor);
  public static IndexerSubsystem m_IndexerSubsystem = new IndexerSubsystem(CANConstants.kIndexerMotor);

  public static PingResponseUltrasonicSubsystem m_pingResponseUltrasonicSubsystem =
    new PingResponseUltrasonicSubsystem(
      UltrasonicConstants.kPingChannel,
      UltrasonicConstants.kEchoChannel,
      UltrasonicConstants.kOffsetToBumper
    );

  // Controller definitions
  private final Joystick leftJoystick = new Joystick(OIConstants.kLeftJoystick);
  private final Joystick rightJoystick = new Joystick(OIConstants.kRightJoystick);
  public static XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  public static double startingHeading;

  double POV_to_double(int pov) {
    double result;
    if (pov == -1) {
      result = 0.0;
    } else if (pov == 0) {
      result = 0.5;
    } else if (pov == 180) {
      result = -0.5;
    } else {
      result = 0.0;
    }
    if (debug) {
      SmartDashboard.putNumber("POV", pov);
      SmartDashboard.putNumber("POV_Out", result);
    }
    return result;
  }
  
  /**
   * Convenience function to create xbox-direction pad rotate buttons.
   * Comprehends alliance to turn to driver's field perspective.
   * @param heading
   * @return xbox-dPad button for turning to the specified heading.
   */
  POVButton dPadButton(int heading) {
    POVButton button = new POVButton(m_driverController, heading);
    button.onTrue(
      new RotateRobotCommand(
          m_robotDrive, 
          Units.degreesToRadians(-heading),
          false
        ).withTimeout(2.0)
    ).debounce(0.2);
    return button;
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {    

    // By default this is not a simulation.
    // For convenience, set the simulation mode to true if this is not linux (ie if it is MacOS or Windows).
    RobotContainer.isSimulation = !Robot.isReal();
    SmartDashboard.putNumber("FW RPM", 0.0);
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands

    // The xBox controller left stick controls translation of the robot.
    // The xBox controller right stick controls the direction the robot is facing (spinning).
    m_robotDrive.setDefaultCommand(
      new RunCommand(
        () -> m_robotDrive.allianceRelativeDrive(
            -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
            true,  true),
        m_robotDrive)
    );
    //m_ShooterSubsystem.setDefaultCommand(new ShooterTest(m_ShooterSubsystem, m_blackBox, m_poseEstimatorSubsystem));
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    
    // Xbox A button drives in a circle
    
      // Xbox Y button resets the robot coorinate system
    new JoystickButton(m_driverController, Button.kY.value).onTrue(new ResetDriveTrainCommand(this));
    new JoystickButton(m_driverController, Button.kX.value).onTrue(new InstantCommand(()->m_poseEstimatorSubsystem.setCurrentPose(new Pose2d(0,0, Rotation2d.kZero))));

    // Xbox start button puts thte robot in fast/speed driving mode.
    new JoystickButton(m_driverController, Button.kStart.value).onTrue(
      new InstantCommand(()->m_robotDrive.setDrivingMode(DrivingMode.SPEED))
    );
    
        // Xbox start button puts thte robot in slow/precision driving mode.
    new JoystickButton(m_driverController, Button.kBack.value).onTrue(
      new InstantCommand(()->m_robotDrive.setDrivingMode(DrivingMode.PRECISION))
    );
    new JoystickButton(m_driverController, Button.kRightBumper.value)
    .onTrue(
      new DriveFacingHub(m_robotDrive, m_poseEstimatorSubsystem, m_driverController)
    );

    new JoystickButton(m_driverController, Button.kA.value).onTrue(
      new InstantCommand(()->m_IntakeSubsystem.runMotor(0.4), m_IntakeSubsystem)
    );
    new JoystickButton(m_driverController, Button.kB.value).onTrue(
      new InstantCommand(()->m_IntakeSubsystem.runMotor(0), m_IntakeSubsystem)
    );

    new JoystickButton(m_driverController, Button.kLeftBumper.value).whileTrue(
      new InstantCommand(()->m_FeederSubsystem.runMotor(10), m_IntakeSubsystem)
    );

    new JoystickButton(leftJoystick, 11).whileTrue(
      new GotoPoseCommand(m_robotDrive, m_poseEstimatorSubsystem, DriveConstants.robotFrontAtPose(m_poseEstimatorSubsystem.getAprilTagPose(19), 0.0) , m_robotDrive.debugAutoConfig)
      );
      Utilities.toSmartDashboard("April Tag 19 pose: ", m_poseEstimatorSubsystem.getAprilTagPose(19));
      Utilities.toSmartDashboard("A19 Robot: ", DriveConstants.robotFrontAtPose(m_poseEstimatorSubsystem.getAprilTagPose(19), 0.0) );





    // POV buttons to point robot to a given heading where 0 is
    // straight downfield from the driver's perspective.
    // These change depending on blue or red alliance.
    POVButton   dPad0 = dPadButton(0);
    POVButton  dPad45 = dPadButton(45);
    POVButton  dPad90 = dPadButton(90);
    POVButton dPad135 = dPadButton(135);
    POVButton dPad180 = dPadButton(180);
    POVButton dPad225 = dPadButton(225);
    POVButton dPad270 = dPadButton(270);
    POVButton dPad315 = dPadButton(315);

    /* Debugging below */
    if (isSimulation()) {
      SmartDashboard.putData("Circle", new DriveCircleThrottle(m_robotDrive, m_poseEstimatorSubsystem, m_robotDrive.circleAutoConfig, 1.0));
      SmartDashboard.putData("DCG2P", new DriveCircle(m_robotDrive, m_poseEstimatorSubsystem, m_robotDrive.circleAutoConfig, 1.145916));
      SmartDashboard.putData("Choreo1", new DriveChoreoPathCommand(m_robotDrive, m_poseEstimatorSubsystem, "CenterShoot", m_robotDrive.circleAutoConfig, 0.1, 0.0));
      SmartDashboard.putData("Y", new ResetDriveTrainCommand(this));
      SmartDashboard.putData("Z", new SimSetRobotPoseCommand(m_robotDrive, m_poseEstimatorSubsystem, new Pose2d(5.5 + 0.145916,4, Rotation2d.kZero)));
      SmartDashboard.putData("R", new RandomRobotPositionCommand(m_robotDrive, m_poseEstimatorSubsystem));
      SmartDashboard.putData("PR", new PointCameraTowardHubCommand(m_robotDrive, m_poseEstimatorSubsystem));
      SmartDashboard.putData("DR", new DriveRobotCommand(m_robotDrive, m_poseEstimatorSubsystem, new Pose2d(2,1,new Rotation2d(0)), false, null));
      SmartDashboard.putData("NV", new DriveWithoutVisionCommand(m_robotDrive, m_poseEstimatorSubsystem, new Pose2d(2,1,new Rotation2d(0)), null));
      SmartDashboard.putData("R0", new RotateRobotCommand(m_robotDrive, Units.degreesToRadians(0),false));
      SmartDashboard.putData("R90", new RotateRobotCommand(m_robotDrive, Units.degreesToRadians(90),false));
      SmartDashboard.putData("R180", new RotateRobotCommand(m_robotDrive, Units.degreesToRadians(180),false));
      SmartDashboard.putData("RG", new RotateRobotG2PCommand(m_robotDrive, m_poseEstimatorSubsystem, Units.degreesToRadians(180),false, null));
      SmartDashboard.putData("RR", new InstantCommand(()->m_robotDrive.drive(0.0, 0.0, 0.1, true)));
      SmartDashboard.putData(
        "G1", new GotoPoseCommand(m_robotDrive, m_poseEstimatorSubsystem, new Pose2d(6, 4, new Rotation2d(Math.PI)), null));
      SmartDashboard.putData(
        "G2", new GotoPoseCommand(m_robotDrive,  m_poseEstimatorSubsystem, new Pose2d(12, 6, new Rotation2d(0)), null));
        SmartDashboard.putData("FW", new FireAtWillCommand(
          m_ShooterSubsystem,
          m_FeederSubsystem,
          m_IndexerSubsystem,
          m_poseEstimatorSubsystem
        ));
        Utilities.toSmartDashboard("April17", m_poseEstimatorSubsystem.getAprilTagPose((17)));
        Utilities.toSmartDashboard("April31", m_poseEstimatorSubsystem.getAprilTagPose((31)));
        SmartDashboard.putData("OP", new GotoPoseCommand(m_robotDrive, m_poseEstimatorSubsystem, DriveConstants.robotFrontAtPose(m_poseEstimatorSubsystem.getAprilTagPose(19), 0.0) , m_robotDrive.defaultAutoConfig));
        SmartDashboard.putData("A29", new GotoPoseCommand(m_robotDrive, m_poseEstimatorSubsystem, DriveConstants.robotRearAtPose(m_poseEstimatorSubsystem.getAprilTagPose(29), 0.0) , m_robotDrive.defaultAutoConfig));
        SmartDashboard.putData("A29L", new GotoPoseCommand(m_robotDrive, m_poseEstimatorSubsystem, DriveConstants.robotLeftAtPose(m_poseEstimatorSubsystem.getAprilTagPose(29), 0.0) , m_robotDrive.defaultAutoConfig));
        SmartDashboard.putData("A29R", new GotoPoseCommand(m_robotDrive, m_poseEstimatorSubsystem, DriveConstants.robotRightAtPose(m_poseEstimatorSubsystem.getAprilTagPose(29), 0.0) , m_robotDrive.defaultAutoConfig));
    } // (isSimulation()
        SmartDashboard.putData("RTest", new ShooterTest(m_ShooterSubsystem, m_poseEstimatorSubsystem));      } // (configureButtonBindings)

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new Autonomous(this);
  }
   
  // Function to align the PoseEstimator pose and the DriveTrain pose.
  // This assumes that the PoseEstimator has a really good estimate.
  // In other words, that it has a recent, accurate view of an Apriltag.
  public void alignDriveTrainToPoseEstimator() {
    m_robotDrive.setAngleDegrees(m_poseEstimatorSubsystem.getCurrentPose().getRotation().getDegrees());
    m_robotDrive.resetOdometry(m_poseEstimatorSubsystem.getCurrentPose());
  }

  /**
   * alignGyroRotationToFieldRotation
   * This function sets the gyro angle based on the alliance (blue or red)
   * The robot should be facing down-field when this command is called.
   * It will set the drive train's location to match the pose estimator.
   * This is based on the hope that the pose estimator has a good fix on the robot position.
   */
  public void alignGyroRotationToFieldRotation() {
    m_robotDrive.setAngleDegrees(AllianceConfigurationSubsystem.robotToFieldRotation().getDegrees());
    m_robotDrive.resetOdometry(
      new Pose2d(
          m_poseEstimatorSubsystem.getCurrentPose().getTranslation()
        , AllianceConfigurationSubsystem.robotToFieldRotation()
      )
    );
  }
   
  /**
   * isSimulation - return true if this is a simulation or false if the robot is running.
   * @return true if this is a simulation, false if this is running a real robot.
   */
  public static boolean isSimulation() {
    return isSimulation;
  }

  public static double startingHeading() {
    return startingHeading;
  }

/**
 * setRobotPose - update the gyro, drivetrain and pose estimated to the supplied pose.
 * 
 * Typically this is used one time at the beginning of autonomous or during simulation
 * to place the robot into known locations on the field.
 * @param pose
 */
  public static void setRobotPose(Pose2d pose) {
    m_robotDrive.setAngleDegrees(pose.getRotation().getDegrees());
    m_robotDrive.resetOdometry(pose);
    m_poseEstimatorSubsystem.setCurrentPose(pose);
  }

}