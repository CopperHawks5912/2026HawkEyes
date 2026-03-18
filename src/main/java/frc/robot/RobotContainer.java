// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DifferentialSubsystem;
import frc.robot.subsystems.feedback.FeedbackSubsystem;
import frc.robot.subsystems.fuel.FuelSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.Elastic;
import frc.robot.util.Utils;

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

  // Auto choosers
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private final SendableChooser<Command> delayChooser = new SendableChooser<>();
  private final HashMap<String, PathPlannerAuto> loadedAutos = new HashMap<>();

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

    // register named commands
    registerNamedCommands();

    // configure auto routines
    configureAutos();

    // configure controller button bindings
    configureButtonBindings();

    // configure state event triggers
    configureEventTriggers();

    // Warm up PathPlanner Path finding to avoid latency on the first path following.
    // This schedule call is safe since it's only called once during initialization.
    // Must be called after initializing the AutoBuilder in the drive subsystem.
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
   
    // silence joystick warnings during testing
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /**
   * Register named commands to be used in PathPlanner autos.
   * Do this before configuring autos. These named commands can be used by exact name in the PathPlanner GUI.
   */
  public void registerNamedCommands() {
    NamedCommands.registerCommand("LAUNCH_FUEL", fuelSubsystem.launchCommand());
    // NamedCommands.registerCommand("LAUNCH_FUEL", fuelSubsystem.launchDistanceCommand(driveSubsystem::getDistanceToAllianceHub));
    NamedCommands.registerCommand("PASS_FUEL", fuelSubsystem.passCommand());
    NamedCommands.registerCommand("INTAKE_FUEL", fuelSubsystem.intakeCommand());
    NamedCommands.registerCommand("EJECT_FUEL", fuelSubsystem.ejectCommand());
    NamedCommands.registerCommand("AIM_AT_HUB", driveSubsystem.aimAtHubCommand());
    NamedCommands.registerCommand("CLIMB_TO_ONE", climberSubsystem.levelOneClimbCommand());
    NamedCommands.registerCommand("CLIMB_TO_TWO", climberSubsystem.levelTwoClimbCommand());
    NamedCommands.registerCommand("PREPARE_TO_CLIMB", climberSubsystem.downToLimitCommand());
  }

  /**
   * Configure the autonomous command chooser and delay chooser and add them to the dashboard.
   * This reads and warms up all the autos found in the /deploy/pathplanner/autos folder.
   */
  private void configureAutos() {
    // Build the auto chooser and add it to the dashboard
    autoChooser.setDefaultOption("No auto", "");
    List<String> autoNames = AutoBuilder.getAllAutoNames();
    for (String autoName : autoNames) {
      // add each auto to the chooser
      autoChooser.addOption(autoName, autoName);

      // pre-load each auto to catch any errors and cache the paths
      Utils.logInfo("Pre-loading auto: " + autoName);
      loadedAutos.put(autoName, new PathPlannerAuto(autoName));
    }
    
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
    driverXbox.y().whileTrue(climberSubsystem.upCommand());

    // climb down while holding B button
    driverXbox.b().whileTrue(climberSubsystem.downCommand());

    // move the climber to the home position
    driverXbox.a().onTrue(climberSubsystem.homeCommand());

    // eject fuel through the intake while holding the X button
    driverXbox.x().whileTrue(fuelSubsystem.ejectCommand());

    // intake fuel from the ground while holding left trigger
    driverXbox.leftTrigger().whileTrue(fuelSubsystem.intakeCommand());

    // pass fuel while holding left bumper
    // driverXbox.leftBumper().whileTrue(fuelSubsystem.passCommand());
    // OR
    // drive in slow mode while holding the left bumper down
    driverXbox.leftBumper()
      .onTrue(driveSubsystem.setSlowModeCommand(true))
      .onFalse(driveSubsystem.setSlowModeCommand(false));

    // launch fuel based on dynamically calculated distance from hub (requires vision)
    driverXbox.rightTrigger().and(visionSubsystem::isEnabled).whileTrue(
      fuelSubsystem.launchDistanceCommand(driveSubsystem::getDistanceToAllianceHub)
    );
    
    // auto-aim at hub when holding right trigger (requires vision)
    driverXbox.rightBumper().and(visionSubsystem::isEnabled).whileTrue(
      driveSubsystem.aimAtHubCommand().andThen(feedbackSubsystem.aimedAtHubCommand())
    );
    
    // launch fuel based on how far the trigger is pressed (non-vision version)
    driverXbox.rightTrigger(0.25).and(() -> !visionSubsystem.isEnabled()).whileTrue(
      fuelSubsystem.launchPowerCommand(driverXbox::getRightTriggerAxis)
    );

    // launch fuel at a fixed power/distance while holding right bumper (non-vision version)
    driverXbox.rightBumper().and(() -> !visionSubsystem.isEnabled()).whileTrue(
      fuelSubsystem.launchCommand()
    );

    // drive in slow mode when in teleop and driver is pressing the POV pad 
    // RobotModeTriggers.teleop().and(() -> driverXbox.getHID().getPOV() != -1).whileTrue(
    //   driveSubsystem.driveCurvatureCommand(
    //     () -> {
    //       int pov = driverXbox.getHID().getPOV();
    //       if (pov == -1) return 0.0;
    //       return Math.cos(Math.toRadians(pov)) * DifferentialConstants.kMaxSlowModeSpeed;
    //     },
    //     () -> {
    //       int pov = driverXbox.getHID().getPOV();
    //       if (pov == -1) return 0.0;
    //       return -Math.sin(Math.toRadians(pov)) * DifferentialConstants.kMaxSlowModeSpeed;
    //     }
    //   )
    // );

    // show various feedbacks for fun
    driverXbox.povUp().and(RobotModeTriggers.disabled()).onTrue(feedbackSubsystem.teamColorsCommand());
    driverXbox.povRight().and(RobotModeTriggers.disabled()).onTrue(feedbackSubsystem.candyCaneCommand());
    driverXbox.povDown().and(RobotModeTriggers.disabled()).onTrue(feedbackSubsystem.funkyDiscoCommand());
    driverXbox.povLeft().and(RobotModeTriggers.disabled()).onTrue(feedbackSubsystem.idleCommand());
  }

  /**
   * Use this to configure triggers that should respond to
   * match phases or other robot/subsystem state changes.
   */
  private void configureEventTriggers() {
    // pre-match init trigger
    RobotModeTriggers.disabled().and(() -> !wasInAuto && !wasInTeleop).onTrue(
      driveSubsystem.setMotorBrakeCommand(false)
    );

    // pre-match periodic trigger
    RobotModeTriggers.disabled().and(() -> !wasInAuto && !wasInTeleop).whileTrue(
      Commands.run(() -> driveSubsystem.updateAutoReadiness(getStartingPose()))
    );

    // autonomous init trigger
    RobotModeTriggers.autonomous().onTrue(Commands.parallel(
      driveSubsystem.autonomousInitCommand(),
      feedbackSubsystem.setGameDataCommand('A'),
      Commands.runOnce(() -> {
        wasInAuto = true;
      })
    ));

    // teleop init trigger
    RobotModeTriggers.teleop().onTrue(Commands.parallel(
      driveSubsystem.teleopInitCommand(),
      Commands.runOnce(() -> {
        wasInTeleop = true;
        Elastic.selectTab("Teleop");
      })
    ));

    // teleop periodic trigger - poll for game data until received
    RobotModeTriggers.teleop().and(() -> gameData == '?').whileTrue(Commands.run(() -> {
      String data = DriverStation.getGameSpecificMessage();
      if (data.length() > 0) {
        char inactiveAlliance = data.charAt(0);
        if (inactiveAlliance == 'R' || inactiveAlliance == 'B') {
          gameData = inactiveAlliance;
        }
      }
    }));

    // set scoring shift based on game data once it's received in teleop
    new Trigger(() -> gameData != '?').onTrue(feedbackSubsystem.setGameDataCommand(gameData));

    // post-match trigger
    RobotModeTriggers.disabled().and(() -> wasInTeleop).onTrue(Commands.parallel(
      driveSubsystem.postMatchCommand(),
      feedbackSubsystem.teamColorsCommand(),
      Commands.runOnce(() -> {
        wasInAuto = false;
        wasInTeleop = false;
        gameData = '?';
      })
    ));

    // show feedback when climber is at upper limit
    climberSubsystem.isAtUpperLimit.and(driverXbox.y()).onTrue(feedbackSubsystem.warningCommand());

    // show feedback when climber is at lower limit
    climberSubsystem.isAtLowerLimit.and(driverXbox.b()).onTrue(feedbackSubsystem.warningCommand());

    // show feedback when climber is at level 1 climb position
    climberSubsystem.isAtLevelOneClimbPosition.onTrue(feedbackSubsystem.infoCommand());

    // show feedback when climber is at level 2 climb position
    climberSubsystem.isAtLevelTwoClimbPosition.onTrue(feedbackSubsystem.infoCommand());

    // show feedback when climber is stalled
    climberSubsystem.isStalled.onTrue(feedbackSubsystem.errorCommand());
  }

  /**
   * Use this to get the starting pose of the currently selected autonomous command
   * @return the starting pose of the selected autonomous command
   */
  private Pose2d getStartingPose() {
    // get the name of the selected auto
    String autoName = autoChooser.getSelected();

    // if no auto is selected, return a default pose (e.g. origin)
    if (autoName == null || autoName.isEmpty()) {
      return new Pose2d();
    }

    // if an auto is selected, then get it from the cached autos
    PathPlannerAuto auto = loadedAutos.get(autoName);
    if (auto == null) {
      return new Pose2d();
    }

    // return the starting pose of the selected auto
    return auto.getStartingPose();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // get the name of the selected auto
    String autoName = autoChooser.getSelected();

    // if no auto is selected, return an empty command
    if (autoName == null || autoName.isEmpty()) {
      return Commands.none();
    }

    // if an auto is selected, then get it from the cached autos
    PathPlannerAuto auto = loadedAutos.get(autoName);
    if (auto == null) {
      return Commands.none();
    }

    // return the selected auto, with the selected delay prepended
    return delayChooser.getSelected().andThen(auto);
  }
}
