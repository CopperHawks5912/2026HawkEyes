// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {
  private final RobotContainer m_robotContainer;
  private Command m_autonomousCommand;

  // Track match state
  private boolean wasInAuto = false;
  private boolean wasInTeleop = false;
  private char gameData = '?';

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Starts recording to data log (ensure a USB stick is plugged in to the RoboRIO)
    // DataLogManager.start();

    // Record both DS control and joystick data
    // DriverStation.startDataLog(DataLogManager.getLog());

    // Instantiate our RobotContainer.
    // This will perform all our button bindings, and put our autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Start the camera server for streaming to the dashboard
    // CameraServer.startAutomaticCapture("DRIVE_FRONT", 0);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
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
    // run post-match code
    if (wasInAuto && wasInTeleop) {
      m_robotContainer.getDriveSubsystem().postMatchInit();
      m_robotContainer.getFeedbackSubsystem().setGameData('?');
      wasInAuto = false;
      wasInTeleop = false;
      gameData = '?';
    }
  }

  /**
   * This function is called periodically while the robot is disabled.
   */
  @Override
  public void disabledPeriodic() {}

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // track that we entered autonomous mode
    wasInAuto = true;

    // initialize the drive subsystem for autonomous mode (resets sensors, sets brake mode, etc)
    m_robotContainer.getDriveSubsystem().autonomousInit();

    // set game data to 'A' for autonomous mode until we get the real data at the start of teleop
    m_robotContainer.getFeedbackSubsystem().setGameData('A');

    // get the selected autonomous command
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {}

  /**
   * This function is called once each time the robot enters Teleop mode.
   */
  @Override
  public void teleopInit() {
    // track that we entered teleop mode
    wasInTeleop = true;

    // initialize the drive subsystem for teleop mode (sets brake mode, etc)
    m_robotContainer.getDriveSubsystem().teleopInit();

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // get game data from the driver station / FMS
    if (gameData == '?') {
      String data = DriverStation.getGameSpecificMessage();
      if (data.length() > 0) {
        char inactiveAlliance = data.charAt(0);
        if (inactiveAlliance == 'R' || inactiveAlliance == 'B') {
          gameData = inactiveAlliance;
          m_robotContainer.getFeedbackSubsystem().setGameData(gameData);
        }
      }
    }
  }

  /**
   * This function is called once each time the robot enters test mode.
   */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {}

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit() {}

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic() {}
}
