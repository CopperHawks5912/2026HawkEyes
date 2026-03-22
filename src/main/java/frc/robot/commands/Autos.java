// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
   * Start:   The back of the robot centered on the left side of the tower.
   * Actions: 1. Drives forward 1m
   * @return Command to run the forwardOneMeter routine
   */
  public Command forwardOneMeter() {
    return Commands.sequence(
      // Reset odometry to the starting pose
      driveSubsystem.resetPoseCommand(new Pose2d(1.518, 4.056, Rotation2d.fromDegrees(0))),

      // Drive backwards 1 meter
      driveSubsystem.driveToPoseCommand(new Pose2d(2.518, 4.056, Rotation2d.fromDegrees(0))), 
      
      // Celebrate with feedback
      feedbackSubsystem.funkyDiscoCommand()
    );
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
  public Command backwardsLaunchReturnClimb() {
    return Commands.sequence(
      // Reset odometry to the starting pose
      driveSubsystem.resetPoseCommand(new Pose2d(1.518, 4.056, Rotation2d.fromDegrees(180))),

      // Drive backwards 1 meter
      driveSubsystem.driveToPoseCommand(new Pose2d(2.518, 4.056, Rotation2d.fromDegrees(180))), 
      
      // Launch fuel for 7.5 seconds
      fuelSubsystem.launchCommand().withTimeout(7.5), 

      // Lower climber all the way
      climberSubsystem.downToLimitCommand(), 

      // Drive forward 1 meter (back to starting position)
      driveSubsystem.driveToPoseCommand(new Pose2d(1.518, 4.056, Rotation2d.fromDegrees(180))), 
      
      // Raise climber to level 1 position
      climberSubsystem.levelOneClimbCommand(),

      // Celebrate with feedback
      feedbackSubsystem.funkyDiscoCommand()
    );
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
  public Command backwardsLaunchReturnClimb2() {
    return Commands.sequence(
      // Reset odometry to the starting pose
      driveSubsystem.resetPoseCommand(new Pose2d(1.518, 4.056, Rotation2d.fromDegrees(180))),

      // Drive backwards 1 meter
      driveSubsystem.driveDistanceCommand(-1.0),

      // Launch fuel for 7.5 seconds
      fuelSubsystem.launchCommand().withTimeout(7.5), 

      // Lower climber all the way
      climberSubsystem.downToLimitCommand(), 

      // Drive forward 1 meter (back to starting position)
      driveSubsystem.driveDistanceCommand(1.0), 
      
      // Raise climber to level 1 position
      climberSubsystem.levelOneClimbCommand(),

      // Celebrate with feedback
      feedbackSubsystem.funkyDiscoCommand()
    );
  }
}
