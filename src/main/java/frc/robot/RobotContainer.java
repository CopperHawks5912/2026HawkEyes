// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.Autos;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DifferentialSubsystem;
import frc.robot.subsystems.feedback.FeedbackSubsystem;
import frc.robot.subsystems.fuel.FuelSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Initialize our controllers
  private final CommandXboxController driverXbox = new CommandXboxController(0);
   
  // The robot's subsystems are defined here...
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final FeedbackSubsystem feedbackSubsystem = new FeedbackSubsystem(driverXbox);
  private final FuelSubsystem fuelSubsystem = new FuelSubsystem();
  private final DifferentialSubsystem driveSubsystem = new DifferentialSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem(driveSubsystem::addVisionMeasurement);

  // Import our autos, passing in any subsystems they require
  private final Autos autos = new Autos(driveSubsystem, fuelSubsystem, climberSubsystem, feedbackSubsystem);
  
  // Auto choosers
  private final SendableChooser<Command> delayChooser = new SendableChooser<>();
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  // Track match state
  private boolean wasInAuto = false;
  private boolean wasInTeleop = false;
  private char gameData = '?';

  /** 
   * The main container constructor for the robot. 
   * Initializes all the things... 
   */
  public RobotContainer() {
    // set our default driving method
    // if we want to switch to arcade drive mode, also update line 207
    driveSubsystem.setDefaultCommand(driveSubsystem.driveCurvatureCommand(
      () -> -1 * driverXbox.getLeftY(),
      () -> -1 * driverXbox.getRightX()
    ));

    // configure auto routines
    configureAutos();

    // configure controller button bindings
    configureButtonBindings();

    // configure state event triggers
    configureEventTriggers();
   
    // silence joystick warnings during testing
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /**
   * Configure the autonomous command chooser and delay chooser and add them to the dashboard.
   * This reads and warms up all the autos specified in the local array.
   */
  private void configureAutos() {
    // Build the auto chooser and add it to the dashboard
    autoChooser.setDefaultOption("No auto", Commands.none());
    autoChooser.addOption("LEFT_FWD_LAUNCH_FWD_CLIMB", autos.leftSideForwardLaunchForwardClimb());
    autoChooser.addOption("RIGHT_FWD_LAUNCH_FWD_CLIMB", autos.rightSideForwardLaunchForwardClimb());
    autoChooser.addOption("FWD_LAUNCH_FWD_CLIMB", autos.forwardLaunchForwardClimb());
    autoChooser.addOption("LAUNCH_5_SECONDS", autos.launchFiveSeconds());
    autoChooser.addOption("FWD_1M", autos.forwardOneMeter());
    autoChooser.addOption("BWD_1M", autos.backwardOneMeter());
    
    // Add auto chooser to dashboard
    SmartDashboard.putData("Auto Command", autoChooser);

    // Configure the available auto delay options
    delayChooser.setDefaultOption("No delay", Commands.none());
    delayChooser.addOption("1.0 second", Commands.waitSeconds(1.0));
    delayChooser.addOption("1.5 seconds", Commands.waitSeconds(1.5));
    delayChooser.addOption("2.0 seconds", Commands.waitSeconds(2.0));
    delayChooser.addOption("2.5 seconds", Commands.waitSeconds(2.5));
    delayChooser.addOption("3.0 seconds", Commands.waitSeconds(3.0));
    delayChooser.addOption("3.5 seconds", Commands.waitSeconds(3.5));
    delayChooser.addOption("4.0 seconds", Commands.waitSeconds(4.0));
    delayChooser.addOption("4.5 seconds", Commands.waitSeconds(4.5));
    delayChooser.addOption("5.0 seconds", Commands.waitSeconds(5.0));
    
    // Add delay chooser to dashboard
    SmartDashboard.putData("Auto Delay", delayChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureButtonBindings() {
    // manually reset odometry & climber home position
    driverXbox.start().onTrue(Commands.parallel(
      driveSubsystem.resetOdometryCommand(),
      climberSubsystem.setHomePositionCommand()
    ));

    // toggles the drive controls inversion (for climbing) when the back button is pressed
    driverXbox.back().onTrue(driveSubsystem.toggleInvertControlsCommand());

    // climb up while holding Y button
    driverXbox.y().onTrue(climberSubsystem.levelOneClimbCommand());

    // climb down while holding B button
    driverXbox.b().onTrue(climberSubsystem.downToLimitCommand());

    // move the climber to the home position
    driverXbox.a().onTrue(climberSubsystem.homeCommand());

    // eject fuel through the intake while holding the X button
    driverXbox.x().whileTrue(fuelSubsystem.ejectCommand());

    // intake fuel from the ground while holding left trigger
    driverXbox.leftTrigger().whileTrue(fuelSubsystem.intakeCommand());

    // drive in slow mode while holding the left bumper down
    driverXbox.leftBumper()
      .onTrue(driveSubsystem.setSlowModeCommand(true))
      .onFalse(driveSubsystem.setSlowModeCommand(false));

    // launch fuel at a fixed power / RPM while holding right bumper (non-vision version)
    driverXbox.rightBumper().and(() -> !visionSubsystem.isEnabled()).whileTrue(
      fuelSubsystem.launchPowerCommand()
    );
    // driverXbox.rightBumper().and(() -> !visionSubsystem.isEnabled()).whileTrue(
    //   fuelSubsystem.launchRpsCommand()
    // );

    // manual climber control with DPAD
    driverXbox.povUp().and(RobotModeTriggers.teleop()).whileTrue(climberSubsystem.upCommand());
    driverXbox.povDown().and(RobotModeTriggers.teleop()).whileTrue(climberSubsystem.downCommand());

    // show various feedbacks for fun
    driverXbox.povUp().and(RobotModeTriggers.disabled()).onTrue(feedbackSubsystem.teamColorsCommand());
    driverXbox.povRight().and(RobotModeTriggers.disabled()).onTrue(feedbackSubsystem.candyCaneCommand());
    driverXbox.povDown().and(RobotModeTriggers.disabled()).onTrue(feedbackSubsystem.funkyDiscoCommand());
    driverXbox.povLeft().and(RobotModeTriggers.disabled()).onTrue(feedbackSubsystem.offCommand());
  }

  /**
   * Use this to configure triggers that should respond to robot/subsystem state changes.
   */
  private void configureEventTriggers() {
    // show feedback when climber is at upper limit
    climberSubsystem.isAtUpperLimit
      .and(RobotModeTriggers.teleop())
      .and(driverXbox.povUp())
      .onTrue(feedbackSubsystem.warningCommand());

    // show feedback when climber is at lower limit
    climberSubsystem.isAtLowerLimit
      .and(RobotModeTriggers.teleop())
      .and(driverXbox.b().or(driverXbox.povDown()))
      .onTrue(feedbackSubsystem.warningCommand());

    // show feedback when climber is at level 1 climb position
    climberSubsystem.isAtLevelOneClimbPosition
      .and(RobotModeTriggers.teleop())
      .onTrue(feedbackSubsystem.infoCommand());

    // show feedback when climber is at level 2 climb position
    climberSubsystem.isAtLevelTwoClimbPosition
      .and(RobotModeTriggers.teleop())
      .onTrue(feedbackSubsystem.infoCommand());

    // show feedback when climber is stalled
    climberSubsystem.isStalled
      .and(RobotModeTriggers.teleop())
      .onTrue(feedbackSubsystem.errorCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // various auto routines
    // return autos.launchFiveSeconds();
    // return autos.forwardOneMeter();
    // return autos.backwardOneMeter();
    // return autos.backwardLaunchReturnClimb();
    return delayChooser.getSelected().andThen(autoChooser.getSelected());
  }

  /**
   * Use this to initialize any subsystems or state for autonomous mode. 
   * This is called once from the main {@link Robot} class before the
   * selected autonomous command is scheduled.
   */
  public void autonomousInit() {
    // ensure tracking flags are reset
    wasInAuto = true;
    wasInTeleop = false;
    gameData = '?';

    // initialize the drive subsystem for autonomous mode (resets sensors, sets brake mode, etc)
    driveSubsystem.autonomousInit();
    climberSubsystem.autonomousInit();

    // set game data to 'A' for autonomous mode until we get the real data at the start of teleop
    feedbackSubsystem.setGameData('A');
  }

  /**
   * Use this to initialize any subsystems or state for teleop mode. 
   * This is called once from the main {@link Robot} class when teleop starts.
   */
  public void teleopInit() {
    // track that we entered teleop mode
    wasInTeleop = true;

    // initialize the drive subsystem for teleop mode (sets brake mode, etc)
    driveSubsystem.teleopInit();
  }

  /**
   * Use this to run any code that should execute once after a match ends (e.g. to clean up subsystems, reset state, etc).
   * This is called once from the main {@link Robot} class when the robot is disabled after being in teleop mode. 
   * If the robot is disabled after being in autonomous mode, this will not be called.
   */
  public void postMatchReset() {
    // run post-match code
    if (wasInTeleop) {
      driveSubsystem.postMatchReset();
      feedbackSubsystem.setGameData('?');
      wasInAuto = false;
      wasInTeleop = false;
      gameData = '?';
    }
  }

  /**
   * Use this to check for game data from the driver station and update subsystems accordingly (e.g. for alliance color).
   * This is called periodically from the main {@link Robot} class while the robot is enabled during teleop mode. 
   */
  public void checkForGameData() {
    // get game data from the driver station / FMS (if we don't already have it)
    if (gameData == '?') {
      String data = DriverStation.getGameSpecificMessage();
      if (data.length() > 0) {
        char inactiveAlliance = data.charAt(0);
        if (inactiveAlliance == 'R' || inactiveAlliance == 'B') {
          gameData = inactiveAlliance;
          feedbackSubsystem.setGameData(gameData);
        }
      }
    }
  }
}
