/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.UltrasonicConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.BlackBoxSubsystem;
import frc.robot.subsystems.DriveSubsystemSRX.DrivingMode;
import frc.robot.subsystems.PingResponseUltrasonicSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.commands.DriveChoreoPathCommand;
import frc.robot.commands.DriveCircle;
import frc.robot.commands.DriveCircleThrottle;
import frc.robot.commands.RandomRobotPosition;
import frc.robot.commands.RotateRobotCommand;
import frc.robot.commands.SwerveToProcessorCommand;
import frc.robot.commands.ResetDriveTrainCommand;
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

  public static BlackBoxSubsystem m_BlackBox = new BlackBoxSubsystem();
  public static PoseEstimatorCamera m_rearCamera = new PoseEstimatorCamera("pv-812", VisionConstants.ROBOT_TO_REAR_CAMERA);
  //public static PoseEstimatorCamera m_frontCamera = new PoseEstimatorCamera("Microsoft_LifeCam_HD-3000", VisionConstants.ROBOT_TO_FRONT_CAMERA);

  public static final PoseEstimatorCamera[] cameras = new PoseEstimatorCamera[]{m_rearCamera/*,m_frontCamera*/};
  public static PoseEstimatorSubsystem m_PoseEstimatorSubsystem = new PoseEstimatorSubsystem( cameras, m_robotDrive);
  private static  boolean isSimulation = (System.getProperty("os.name").equals("Mac OS X"));
  public static PreussDriveSimulation m_preussDriveSimulation = new PreussDriveSimulation(m_PoseEstimatorSubsystem);
  private static boolean debug = true; // To enable debugging in this module, change false to true.

  public static PingResponseUltrasonicSubsystem m_PingResponseUltrasonicSubsystem =
    new PingResponseUltrasonicSubsystem(
      UltrasonicConstants.kPingChannel,
      UltrasonicConstants.kEchoChannel,
      UltrasonicConstants.kOffsetToBumper
    );

  // Controller definitions
  private final Joystick leftJoystick = new Joystick(OIConstants.kLeftJoystick);
  private final Joystick rightJoystick = new Joystick(OIConstants.kRightJoystick);
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
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
    RobotContainer.isSimulation = !(System.getProperty("os.name").equals("Linux"));
    TrajectoryPlans.buildAutoTrajectories(); 

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
    new JoystickButton(m_driverController, Button.kA.value)
    .whileTrue(
      new DriveCircleThrottle(m_robotDrive, m_PoseEstimatorSubsystem, m_robotDrive.circleAutoConfig, 1.0));
  
      // Xbox Y button resets the robot coorinate system
    new JoystickButton(m_driverController, Button.kY.value).onTrue(new ResetDriveTrainCommand(this));
    new JoystickButton(m_driverController, Button.kX.value).onTrue(new InstantCommand(()->m_PoseEstimatorSubsystem.setCurrentPose(new Pose2d(0,0, Rotation2d.kZero))));

    // Xbox start button puts thte robot in fast/speed driving mode.
    new JoystickButton(m_driverController, Button.kStart.value).onTrue(
      new InstantCommand(()->m_robotDrive.setDrivingMode(DrivingMode.SPEED))
    );
    
        // Xbox start button puts thte robot in slow/precision driving mode.
    new JoystickButton(m_driverController, Button.kBack.value).onTrue(
      new InstantCommand(()->m_robotDrive.setDrivingMode(DrivingMode.PRECISION))
    );


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
      SmartDashboard.putData("Circle", new DriveCircleThrottle(m_robotDrive, m_PoseEstimatorSubsystem, m_robotDrive.circleAutoConfig, 1.0));
      SmartDashboard.putData("DCG2P", new DriveCircle(m_robotDrive, m_PoseEstimatorSubsystem, m_robotDrive.circleAutoConfig, 1.145916));
      SmartDashboard.putData("Choreo1", new DriveChoreoPathCommand(m_robotDrive, m_PoseEstimatorSubsystem, "Blue 1 Meter", m_robotDrive.circleAutoConfig, 0.1, 0.0));
      SmartDashboard.putData("Choreo2", new DriveChoreoPathCommand(m_robotDrive, m_PoseEstimatorSubsystem, "Low to AT22", m_robotDrive.circleAutoConfig, 1.0, 1.0));
      SmartDashboard.putData("Choreo3", new DriveChoreoPathCommand(m_robotDrive, m_PoseEstimatorSubsystem, "PID test", m_robotDrive.circleAutoConfig, 1.0, 1.0));
      SmartDashboard.putData("Y", new ResetDriveTrainCommand(this));
      SmartDashboard.putData("Z", new InstantCommand(()->m_PoseEstimatorSubsystem.setCurrentPose(new Pose2d(5.5 + 0.145916,4, Rotation2d.kZero))));
      SmartDashboard.putData("TT", new SwerveToProcessorCommand(m_robotDrive, m_PoseEstimatorSubsystem).withTimeout(15.0));
      SmartDashboard.putData("R", new RandomRobotPosition(m_PoseEstimatorSubsystem));

    } // (isSimulation()
  } // (configureButtonBindings)

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
    // Set the gyro angle to match the pose estimator 
    // compensating for the placement of the camera on the robot.
    /*
    m_robotDrive.setAngleDegrees( 
      MathUtil.inputModulus(
        m_PoseEstimatorSubsystem.getCurrentPose().getRotation().getDegrees()+VisionConstants.CAMERA_TO_ROBOT.getRotation().toRotation2d().getDegrees()
        ,-180
        , 180
      )
    );
    */
    // Update the drive trains X, Y, and robot orientation to match the pose estimator.
    m_robotDrive.resetOdometry(m_PoseEstimatorSubsystem.getCurrentPose());
  }

  /**
   *  This function sets the gyro angle based on the alliance (blue or red)
   * and the assumed starting position of the robot on the field.
   * The current assumption is that the robot will be placed with it's back to the
   * alliance wall.  The "Y" coordinates of the robot will be determined by the
   * PoseEstimator once an april tag is captured by the vision system.
   */
   public void setGyroAngleToStartMatch() {
    boolean isBlueAlliance = Utilities.isBlueAlliance(); // From the Field Management system.
    if (isBlueAlliance) {
      startingHeading = 0.0;
      m_robotDrive.setAngleDegrees(startingHeading);
      m_robotDrive.resetOdometry(
        new Pose2d(
           DriveConstants.kBackToCenterDistance // Assume robot is back to the starting wall
          ,FieldConstants.yCenter               // Pick center of the field since we dont know where we will start.
          ,new Rotation2d(Units.degreesToRadians(startingHeading)) // Facing toward the field.
        )
      );
    } else {
      startingHeading = 180.0;
      m_robotDrive.setAngleDegrees(startingHeading);
      m_robotDrive.resetOdometry(
        new Pose2d(
           FieldConstants.xMax - DriveConstants.kBackToCenterDistance // Assume robot is back to the starting wall
          ,FieldConstants.yCenter                                     // Pick center of the field since we dont know where we will start.
          ,new Rotation2d(Units.degreesToRadians(startingHeading))    // Facing toward the field.
        )
      );
    }
   }
   /**
   *  This function sets the gyro angle based on the alliance (blue or red)
   * and the assumed starting position of the robot on the field.
   * The current assumption is that the robot will be placed with it's back 
   * close to the Processor wall. The X coodinate will be calculate based on the
   * assumption that the robot is just inside the starting box.
   */
  public void setGyroAngleToStartMatchProcessor() {
    double ProcessorZoneDepth = Units.inchesToMeters(13);
    double startingBoxDepth = Units.inchesToMeters(76.1);
    double startingRobotCenterY = FieldConstants.yMax - ProcessorZoneDepth - DriveConstants.kBackToCenterDistance;
    double startingRobotCenterX = FieldConstants.xMin + startingBoxDepth - DriveConstants.kRobotWidth/2.0 - Units.inchesToMeters(2.0)/* tape */;

    boolean isBlueAlliance = Utilities.isBlueAlliance(); // From the Field Management system.
    if (isBlueAlliance) {
      startingHeading = 90; // Facing source wall
      m_robotDrive.setAngleDegrees(startingHeading);
      m_robotDrive.resetOdometry(
        new Pose2d(
           startingRobotCenterX     // Just inside the starting box near the Processor
          ,startingRobotCenterY     // Just inside the starting box near the Processor
          ,new Rotation2d(Units.degreesToRadians(startingHeading)) // Facing toward the field.
        )
      );
    } else {
      startingHeading = 90.0; // Facing source wall
      m_robotDrive.setAngleDegrees(startingHeading);
      m_robotDrive.resetOdometry(
        new Pose2d(
           FieldConstants.xMax - startingRobotCenterX     // Just inside the starting box near the Processor
          ,startingRobotCenterY                           // Just inside the starting box near the Processor
          ,new Rotation2d(Units.degreesToRadians(startingHeading))    // Facing toward the field.
        )
      );
    }
   }

   /**
    * isSimulation - return true if this is a simulation or false if the robot is running.
    */
    public static boolean isSimulation() {
      return isSimulation;
    }

    public static double startingHeading() {
      return startingHeading;
    }

}