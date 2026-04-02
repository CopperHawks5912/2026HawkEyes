// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DifferentialSubsystem;
import frc.robot.subsystems.feedback.FeedbackSubsystem;
import frc.robot.subsystems.fuel.FuelSubsystem;

/**
 * This class is where you can define all of your autonomous commands. These
 * can be simple commands or complex command groups that run multiple commands
 * in sequence or in parallel. 
 */
public class Autos {
  private final DifferentialSubsystem driveSubsystem;
  private final FuelSubsystem fuelSubsystem;
  private final ClimberSubsystem climberSubsystem;
  private final FeedbackSubsystem feedbackSubsystem;

  /**
   * Constructor for Autos command group. This is where you should pass in any 
   * subsystems you will use in your autonomous commands.
   * @param driveSubsystem The drive subsystem
   * @param fuelSubsystem The fuel subsystem
   * @param climberSubsystem The climber subsystem
   * @param feedbackSubsystem The feedback subsystem
   */
  public Autos(
    DifferentialSubsystem driveSubsystem, 
    FuelSubsystem fuelSubsystem,
    ClimberSubsystem climberSubsystem,
    FeedbackSubsystem feedbackSubsystem
  ) {
    this.driveSubsystem = driveSubsystem;
    this.fuelSubsystem = fuelSubsystem;
    this.climberSubsystem = climberSubsystem;
    this.feedbackSubsystem = feedbackSubsystem;
  }

  /**
   * Start:   The launcher of the robot facing the hub.
   * Actions: 1. Launches fuel for 5 seconds
   * @return Command to run the launchFiveSeconds routine
   */
  public Command launchFiveSeconds() {
    return  fuelSubsystem.launchPowerCommand().withTimeout(5.0).finallyDo(fuelSubsystem::stop)
    .withName("launchFiveSeconds");
  }

  /**
   * Start:   The back of the robot centered on the left side of the tower.
   * Actions: 1. Drives forward 1m
   * @return Command to run the forwardOneMeter routine
   */
  public Command forwardOneMeter() {
    return driveSubsystem.driveDistanceCommand(1.0)
      .andThen(feedbackSubsystem.idleCommand())
      .withName("forwardOneMeter");
  }

  /**
   * Start:   The back of the robot centered on the left side of the tower.
   * Actions: 1. Drives backward 1m
   * @return Command to run the backwardOneMeter routine
   */
  public Command backwardOneMeter() {
    return driveSubsystem.driveDistanceCommand(-1.0)
      .andThen(feedbackSubsystem.idleCommand())
      .withName("backwardOneMeter");
  }

  /**
   * Start:   The back of the robot touching the hub, aligned with the left side of the tower.
   * Actions: 1. Drives forward 63.5 inches
   *          2. Launches fuel for 5 seconds
   *          3. Lowers climber all the way
   *          4. Drives forward 18.5 inches (just touching tower)
   *          5. Raises climber to level 1 position
   * @return Command to run the leftSideForwardLaunchForwardClimb routine
   */
  public Command leftSideForwardLaunchForwardClimb() {
    return Commands.sequence(
      // Drive forward 63.5 inches
      driveSubsystem.driveDistanceCommand(Units.inchesToMeters(63.5), 0.20),

      Commands.parallel(
        // Launch fuel for 5.0 seconds
        fuelSubsystem.launchPowerCommand().withTimeout(5.0).finallyDo(fuelSubsystem::stop),

        // Wait a brief period and then lower the climber all the way
        Commands.waitSeconds(2.5).andThen(climberSubsystem.downToLimitCommand())
      ),

      // Drive forward 18.5 inches (just touching tower)
      driveSubsystem.driveDistanceCommand(Units.inchesToMeters(18.5), 0.20),
      
      // Raise climber to level 1 position
      climberSubsystem.levelOneClimbCommand(),

      // Show feedback
      feedbackSubsystem.idleCommand()
    )    
    .withName("leftSideForwardLaunchForwardClimb");
  }

  /**
   * Start:   The back of the robot touching the hub, aligned with the right side of the tower.
   * Actions: 1. Drives forward 63.5 inches
   *          2. Turns to face the center of the hub
   *          3. Launches fuel for 5 seconds
   *          4. Lowers climber all the way
   *          5. Turns to face the straight ahead direction again
   *          6. Drives forward 18.5 inches (just touching tower)
   *          7. Raises climber to level 1 position
   * @return Command to run the rightSideForwardLaunchForwardClimb routine
   */
  public Command rightSideForwardLaunchForwardClimb() {
    return Commands.sequence(
      // Drive forward 63.5 inches
      driveSubsystem.driveDistanceCommand(Units.inchesToMeters(63.5), 0.20),

      // Turn to face the center of the hub
      driveSubsystem.turnToHeadingCommand(5.0),
      // driveSubsystem.turnDegreesCommand(5.0),

      Commands.parallel(
        // Launch fuel for 5.0 seconds
        fuelSubsystem.launchPowerCommand().withTimeout(5.0).finallyDo(fuelSubsystem::stop),

        // Wait a brief period and then lower the climber all the way
        Commands.waitSeconds(2.5).andThen(climberSubsystem.downToLimitCommand())
      ),

      // Turn to face the straight ahead direction again
      driveSubsystem.turnToHeadingCommand(0),
      // driveSubsystem.turnDegreesCommand(-5.0),

      // Drive forward 18.5 inches (just touching tower)
      driveSubsystem.driveDistanceCommand(Units.inchesToMeters(18.5), 0.20),
      
      // Raise climber to level 1 position
      climberSubsystem.levelOneClimbCommand(),

      // Show feedback
      feedbackSubsystem.idleCommand()
    )    
    .withName("rightSideForwardLaunchForwardClimb");
  }

  /**
   * Start:   The back of the robot touching the hub, aligned with the left side of the tower.
   * Actions: 1. Turn on slow mode
   *          2. Drives backward 61 inches
   *          3. Launches fuel for 5 seconds
   *          4. Turn off slow mode
   * @return Command to run the moveAndShoot routine
   */
  public Command moveAndShootCommand() {
    return Commands.sequence(
      // // Turn on slow mode
      driveSubsystem.setSlowModeCommand(true),

      // Drive backwards 1 meter
      driveSubsystem.driveDistanceCommand(Units.inchesToMeters(61), 0.50),

      // Launch fuel for 5.0 seconds
      fuelSubsystem.launchPowerCommand().withTimeout(5.0).finallyDo(fuelSubsystem::stop),

      // // Lower climber all the way
      // climberSubsystem.downToLimitCommand(), 

      // // Drive forward 1 meter (back to starting position)
      // driveSubsystem.driveDistanceCommand(Units.inchesToMeters(18.5), 0.405),
      
      // // Raise climber to level 1 position
      // climberSubsystem.levelOneClimbCommand(),

      // Turn off slow mode
      driveSubsystem.setSlowModeCommand(false)
    )    
    .withName("moveAndShootCommand");
  }

  /**
   * Start:   The front of the robot facing & centered on the left side of the tower.
   * Actions: 1. Turn on slow mode
   *          2. Drives in reverse 18.5 inches
   *          3. Launches fuel for 5 seconds
   *          4. Lowers climber all the way
   *          5. Drives forward 18.5 inches (back to starting position)
   *          6. Raises climber to level 1 position
   *          7. Turn off slow mode
   * @return Command to run the backwardLaunchReturnClimb routine
   */
  public Command forwardLaunchForwardClimb() {
    return Commands.sequence(
      // // Turn on slow mode
      driveSubsystem.setSlowModeCommand(true),

      // Drive backwards 1 meter
      driveSubsystem.driveDistanceCommand(Units.inchesToMeters(63.5), 0.50),

      Commands.parallel(
        // Launch fuel for 5.0 seconds
        fuelSubsystem.launchPowerCommand().withTimeout(5.0).finallyDo(fuelSubsystem::stop),

        // wait AND then lower the climber
        Commands.waitSeconds(2.0).andThen(climberSubsystem.downToLimitCommand())
      ),

      // Drive forward 1 meter (back to starting position)
      driveSubsystem.driveDistanceCommand(Units.inchesToMeters(15.5), 0.425).withTimeout(2.25),
      
      // Raise climber to level 1 position
      climberSubsystem.levelOneClimbCommand(),

      // Turn off slow mode
      driveSubsystem.setSlowModeCommand(false)
    )    
    .withName("forwardLaunchForwardClimb");
  }
}
