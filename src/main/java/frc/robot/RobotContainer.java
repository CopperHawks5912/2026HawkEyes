// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.feedback.FeedbackSubsystem;
import frc.robot.subsystems.fuel.FuelSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.Elastic;
import frc.robot.subsystems.drive.DifferentialSubsystem;

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
  @SuppressWarnings("unused")
  private final VisionSubsystem visionSubsystem = new VisionSubsystem(driveSubsystem::addVisionMeasurement);

  // Auto choosers
  private SendableChooser<String> autoChooser = new SendableChooser<>();
  private SendableChooser<Command> delayChooser = new SendableChooser<>();
  
  // Cache the selected auto to avoid repeatedly loading path files while disabled
  private PathPlannerAuto selectedAuto = null;
  private boolean hasGameData = false;

  /** 
   * The container for the robot. 
   * Contains subsystems, IO devices, and commands. 
   */
  public RobotContainer() {
    // set our default driving method (arcade - differential drive)
    driveSubsystem.setDefaultCommand(driveSubsystem.driveArcadeCommand(
      () -> -1 * driverXbox.getLeftY(),
      () -> -1 * driverXbox.getRightX()
    ));

    // register our named commands
    registerNamedCommands();

    // configure our auto routines
    configureAutos();

    // configure our controller bindings
    configureBindings();

    // silence joystick warnings during testing
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /**
   * Register named commands to be used in PathPlanner autos.
   * Do this before the creation of any PathPlanner Autos or Paths. 
   * It is recommended to do this in RobotContainer, after subsystem 
   * initialization, but before the creation of any other commands.
   */
  public void registerNamedCommands() {
    NamedCommands.registerCommand("LAUNCH_FUEL", fuelSubsystem.launchCommand(() -> driveSubsystem.getDistanceToAllianceHub()));
    NamedCommands.registerCommand("PASS_FUEL", fuelSubsystem.passCommand());
    NamedCommands.registerCommand("INTAKE_FUEL", fuelSubsystem.intakeCommand());
    NamedCommands.registerCommand("EJECT_FUEL", fuelSubsystem.ejectCommand());
    NamedCommands.registerCommand("AIM_AT_HUB", driveSubsystem.aimAtHubCommand());
    NamedCommands.registerCommand("CLIMB", climberSubsystem.upToLimitCommand());
    NamedCommands.registerCommand("PREPARE_TO_CLIMB", climberSubsystem.downToLimitCommand());
  }

  /**
   * Register named commands and configure the autonomous command chooser.
   * This will build the auto chooser using the AutoBuilder class, 
   * which pulls in all autos defined in the PathPlanner deploy folder.
   */
  private void configureAutos() {
    // Build the auto chooser and add it to the dashboard
    autoChooser.setDefaultOption("No auto", "");
    List<String> autoNames = AutoBuilder.getAllAutoNames();
    for (String autoName : autoNames) {
      // add each auto to the chooser
      autoChooser.addOption(autoName, autoName);

      // pre-load each auto to catch any errors and cache the paths
      new PathPlannerAuto(autoName).cancel();
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
  private void configureBindings() {
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

    // Pass fuel while holding left bumper
    // Automatically spins up, then feeds when ready
    driverXbox.leftBumper().whileTrue(fuelSubsystem.passCommand());

    // Auto-aim at hub when pressing right trigger
    driverXbox.rightTrigger().whileTrue(
      driveSubsystem.aimAtHubCommand().andThen(feedbackSubsystem.aimedAtHubCommand())
    );

    // Launch fuel while holding right bumper
    // Automatically calculates distance, spins up, feeds when ready
    driverXbox.rightBumper().whileTrue(
      fuelSubsystem.launchCommand(() -> driveSubsystem.getDistanceToAllianceHub())
    );

    // Override above bindings with bindings to run SysId commands
    if (DriverStation.isTest()) {
      driverXbox.y().whileTrue(driveSubsystem.sysIdQuasistaticCommand(SysIdRoutine.Direction.kForward));
      driverXbox.b().whileTrue(driveSubsystem.sysIdQuasistaticCommand(SysIdRoutine.Direction.kReverse));
      driverXbox.x().whileTrue(driveSubsystem.sysIdDynamicCommand(SysIdRoutine.Direction.kForward));
      driverXbox.a().whileTrue(driveSubsystem.sysIdDynamicCommand(SysIdRoutine.Direction.kReverse));
      driverXbox.leftTrigger().whileTrue(fuelSubsystem.sysIdQuasistaticCommand(SysIdRoutine.Direction.kForward));
      driverXbox.leftBumper().whileTrue(fuelSubsystem.sysIdQuasistaticCommand(SysIdRoutine.Direction.kReverse));
      driverXbox.rightTrigger().whileTrue(fuelSubsystem.sysIdDynamicCommand(SysIdRoutine.Direction.kForward));
      driverXbox.rightBumper().whileTrue(fuelSubsystem.sysIdDynamicCommand(SysIdRoutine.Direction.kReverse));
    }
  }

  /**
   * Use this to pass the starting pose to the main {@link Robot} class.
   * @return the starting pose of the selected autonomous command
   */
  private Pose2d getStartingPose() {
    // get the name of the selected auto from the chooser
    String autoName = autoChooser.getSelected();

    // if no auto is selected, return a default pose at the origin
    if (autoName == null || autoName.isEmpty()) {
      return new Pose2d();
    }

    // cache the selected auto to avoid repeatedly loading path files while disabled
    if (selectedAuto == null || !autoName.equals(selectedAuto.getName())) {
      selectedAuto = new PathPlannerAuto(autoName);
    }

    // return the cached starting pose
    return selectedAuto.getStartingPose();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // get the selected auto from the chooser if it 
    // wasn't already cached in getStartingPose()
    if (selectedAuto == null) {
      String autoName = autoChooser.getSelected();
      if (autoName != null && !autoName.isEmpty()) {
        selectedAuto = new PathPlannerAuto(autoName);
      }
    }

    // get the selected auto, this was cached in getStartingPose() 
    // to avoid repeatedly loading path files while disabled
    Command autoCommand = selectedAuto != null ? selectedAuto : Commands.none();
    
    // get the selected delay command and then call the selected auto
    return delayChooser.getSelected().andThen(autoCommand);
  }

  /**
   * This function is called once each time the robot enters autonomous mode.
   */
  public void autonomousInit() {
    hasGameData = false;
    CommandScheduler.getInstance().schedule(
      driveSubsystem.autonomousInitCommand(),
      feedbackSubsystem.scoringShiftCommand('A')
    );
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  public void teleopInit() {
    CommandScheduler.getInstance().schedule(driveSubsystem.teleopInitCommand());
    Elastic.selectTab("Teleop");
  }

  /**
   * This function is called periodically during teleop.
   */
  public void teleopPeriodic() {
    // Poll for the game data and pass it to the feedback subsystem.
    // Stop further polling once we have valid game data.
    if (!hasGameData) {
      String gameData = DriverStation.getGameSpecificMessage();
      if (gameData.length() > 0) {
        char inactiveAlliance = gameData.charAt(0);
        if (inactiveAlliance == 'R' || inactiveAlliance == 'B') {
          hasGameData = true;
          CommandScheduler.getInstance().schedule(
            feedbackSubsystem.scoringShiftCommand(inactiveAlliance)
          );
        }
      }
    }
  }

  /**
   * This function is called periodically before the start of each match.
   * It can be used to update the dashboard with information about the 
   * selected autonomous routine, robot pose readiness, etc.
   */
  public void preMatch() {
    driveSubsystem.updateAutoReadiness(getStartingPose());
  }

  /**
   * This function is called once at the end of each match.
   */
  public void endOfMatch() {
    hasGameData = false;
    CommandScheduler.getInstance().schedule(
      driveSubsystem.endOfMatchCommand(),
      feedbackSubsystem.teamColorsCommand()
    );
  }
}
