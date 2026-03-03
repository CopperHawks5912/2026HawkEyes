// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private boolean wasInAuto;
  private boolean wasInTeleop;

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

    // Send the remaing match time to the dashboard
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    if (isPostMatch()) {
      wasInAuto = false;
      wasInTeleop = false;
      m_robotContainer.onPostMatch();
    }
  }

  /**
   * This function is called periodically while the robot is disabled.
   */
  @Override
  public void disabledPeriodic() {
    if (isPreMatch()) {
      m_robotContainer.onPreMatch();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // Set autonomous tracking flag
    wasInAuto = true;

    // Run the autonomous init command for all subsystems
    m_robotContainer.onAutonomousInit();

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
    // Set teleop tracking flag
    wasInTeleop = true;

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // Run the teleop init command for all subsystems
    m_robotContainer.onTeleopInit();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    m_robotContainer.onTeleopPeriodic();
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

  /**
   * Check if the match has not started yet by seeing if we were not in auto or teleop.
   * @return true if the match has not started, false otherwise
   */
  private boolean isPreMatch() {
    return !wasInAuto && !wasInTeleop;
  }

  /**
   * Check if the match has ended by seeing if we were in teleop and the match time is 0 or less.
   * @return true if the match has ended, false otherwise
   */
  private boolean isPostMatch() {
    return wasInTeleop;
  }
}
