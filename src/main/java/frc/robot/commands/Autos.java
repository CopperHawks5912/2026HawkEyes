// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    return Commands.sequence(
      // Launch fuel for 5.0 seconds
      fuelSubsystem.launchCommand().withTimeout(5.0).finallyDo(fuelSubsystem::stopCommand),

      // Celebrate with feedback
      feedbackSubsystem.funkyDiscoCommand()
    )    
    .withName("launchFiveSeconds");
  }

  /**
   * Start:   The back of the robot centered on the left side of the tower.
   * Actions: 1. Drives forward 1m
   * @return Command to run the forwardOneMeter routine
   */
  public Command forwardOneMeter() {
    return Commands.sequence(
      // Drive forward 1 meter
      driveSubsystem.driveDistanceCommand(1), 
      
      // Celebrate with feedback
      feedbackSubsystem.funkyDiscoCommand()
    )
    .withName("forwardOneMeter");
  }

  /**
   * Start:   The back of the robot centered on the left side of the tower.
   * Actions: 1. Drives backward 1m
   * @return Command to run the backwardOneMeter routine
   */
  public Command backwardOneMeter() {
    return Commands.sequence(
      // Drive backward 1 meter
      driveSubsystem.driveDistanceCommand(-1), 
      
      // Celebrate with feedback
      feedbackSubsystem.funkyDiscoCommand()
    )
    .withName("backwardOneMeter");
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
  public Command backwardLaunchReturnClimb() {
    return Commands.sequence(
      // // Turn on slow mode
      driveSubsystem.setSlowModeCommand(true),

      // Drive backwards 1 meter
      driveSubsystem.driveDistanceCommand(Units.inchesToMeters(-18.5)),

      // Launch fuel for 5.0 seconds
      fuelSubsystem.launchCommand().withTimeout(5.0).finallyDo(fuelSubsystem::stopCommand),

      // Lower climber all the way
      climberSubsystem.downToLimitCommand(), 

      // Drive forward 1 meter (back to starting position)
      driveSubsystem.driveDistanceCommand(Units.inchesToMeters(18.0)), 
      
      // Raise climber to level 1 position
      climberSubsystem.levelOneClimbCommand(),

      // Turn off slow mode
      driveSubsystem.setSlowModeCommand(false),

      // Celebrate with feedback
      feedbackSubsystem.funkyDiscoCommand()
    )    
    .withName("backwardLaunchReturnClimb");
  }

  /**
   * Start:   The front of the robot facing & centered on the left side of the tower.
   * Actions: 1. Drives in reverse 1m
   *          2. Launches fuel for 5 seconds
   *          3. Lowers climber all the way
   *          4. Drives forward 1m (back to starting position)
   *          5. Raises climber to level 1 position
   * @return Command to run the backwardsLaunchReturnClimb routine
   */
  public Command unused1() {
    return Commands.sequence(
      // Reset odometry to the starting pose
      driveSubsystem.resetPoseCommand(new Pose2d(1.518, 4.056, Rotation2d.fromDegrees(180))),

      // Drive backwards 1 meter
      driveSubsystem.driveToPoseCommand(new Pose2d(2.518, 4.056, Rotation2d.fromDegrees(180))), 
      
      // Launch fuel for 7.5 seconds
      fuelSubsystem.launchCommand().withTimeout(7.5).finallyDo(fuelSubsystem::stopCommand), 

      // Lower climber all the way
      climberSubsystem.downToLimitCommand(), 

      // Drive forward 1 meter (back to starting position)
      driveSubsystem.driveToPoseCommand(new Pose2d(1.518, 4.056, Rotation2d.fromDegrees(180))), 
      
      // Raise climber to level 1 position
      climberSubsystem.levelOneClimbCommand(),

      // Celebrate with feedback
      feedbackSubsystem.funkyDiscoCommand()
    )
    .withName("unused1");
  }

  /**
   * Start:   The front of the robot facing & centered on the left side of the tower.
   * Actions: 1. Drives in reverse 1m
   *          2. Launches fuel for 5 seconds
   *          3. Lowers climber all the way
   *          4. Drives forward 1m (back to starting position)
   *          5. Raises climber to level 1 position
   * @return Command to run the backwardsLaunchReturnClimb routine
   */
  public Command unused2() {
    return Commands.sequence(
      // Reset odometry to the starting pose
      driveSubsystem.resetPoseCommand(new Pose2d(1.518, 4.056, Rotation2d.fromDegrees(180))),

      // Drive backwards 1 meter
      driveSubsystem.driveDistanceCommand(Units.inchesToMeters(-18.5)),

      // Launch fuel for 7.5 seconds
      fuelSubsystem.launchCommand().withTimeout(6.0).finallyDo(fuelSubsystem::stopCommand), 

      // Lower climber all the way
      climberSubsystem.downToLimitCommand(), 

      // Drive forward 1 meter (back to starting position)
      driveSubsystem.driveDistanceCommand( Units.inchesToMeters(18.5)), 
      
      // Raise climber to level 1 position
      climberSubsystem.levelOneClimbCommand(),

      // Celebrate with feedback
      feedbackSubsystem.funkyDiscoCommand()
    )    
    .withName("unused2");
  }
}
