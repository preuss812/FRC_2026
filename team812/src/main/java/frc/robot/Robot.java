/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystemSRX.DrivingMode;
//import frc.robot.Constants.OIConstants;
//import frc.robot.subsystems.DriveSubsystemSRX;
import edu.wpi.first.cameraserver.CameraServer;

// import frc.robot.RobotContainer;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  public static NetworkTable nttable;
  static int i = 0;
  private static boolean debug = true;
  private static boolean usingCameraServer = true; // Set to true to enable the usb camera plugged into the roboRIO.
  public static SendableChooser<Integer> autoChooser = new SendableChooser<>();

  /*
  private boolean drivingSwitchPosition = false; // Assume the switch is not set (SPEED mode).
  private boolean endGameSwitchPosition = false; // Assume we are not starting in endgame.
  private boolean blackBox = false; // For now we are not using the black box.
  private boolean powerDistribution = false; // For now we are not tracking power distribution.
  */

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
    nttable = ntinst.getTable("0team812");

    if (usingCameraServer) {
      CameraServer.startAutomaticCapture(0); // 2024 forward facing camera for driverstation
    }

    m_robotContainer = new RobotContainer();
    if (debug) SmartDashboard.putData(CommandScheduler.getInstance()); // This puts running commands on the shuffleboard.
    addPeriodic(() -> Utilities.setAlliance(), 1.0 );

    // Add a dropdown menu to select the autonomous plan.

    //SmartDashboard.putNumber("AutoStartDelay", 0.0);  // This puts up a place on the dashboard we can use to modify autonomous.
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    //try {
      //int autoMode = autoChooser.getSelected();
    
      // These next 2 are redundant and just for debug:
      //SmartDashboard.putNumber("AutoMode", autoMode);
      //SmartDashboard.putString("AutoModeText", TrajectoryPlans.autoNames.get(autoMode));
    //}
    //catch (Exception e) {}

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // Ensure that at startup, the robot is in a known speed mode.
    RobotContainer.m_robotDrive.setDrivingMode(DrivingMode.SPEED);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
