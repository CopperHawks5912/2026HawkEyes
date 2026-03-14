// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /**
   * Analog IO contants
   */
  public static final class AnalogConstants {}

  /**
   * CAN bus IO contants
   */
  public static final class CANConstants {
    // NavX 3 gyro (recommended to be on ID 0 for better stability)
    public static final int kGyroID                             = 9;

    // Fuel subsystem motors
    public static final int kLeftIntakeLauncherMotorID          = 1;
    public static final int kRightIntakeLauncherMotorID         = 2;
    public static final int kFeederMotorID                      = 3;

    // Climber subsystem motor
    public static final int kClimberMotorID                     = 8;

    // Differential drive subsystem motors
    public static final int kLeftDifferentialLeaderMotorID      = 5;
    public static final int kLeftDifferentialFollowerMotorID    = 7;
    public static final int kRightDifferentialLeaderMotorID     = 6;
    public static final int kRightDifferentialFollowerMotorID   = 4;
    public static final int kPowerDistributionHubID             = 10;
  }
  
  /**
   * Digital IO constants
   */
  public static final class DIOConstants {}

  /**
   * PWM IO constants
   */
  public static class PWMConstants {
    public static final int kLEDStringID = 0;
  }

  /**
   * Field constants
   */
  public static class FieldConstants {
    public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    public static final double kFieldLengthMeters = Units.inchesToMeters(651.22); // meters
    public static final double kFieldWidthMeters = Units.inchesToMeters(317.69); // meters
    public static final Translation2d kBlueHubCenter = new Translation2d(Units.inchesToMeters(158.84), Units.inchesToMeters(182.11));
    public static final Translation2d kRedHubCenter = new Translation2d(Units.inchesToMeters(158.84), Units.inchesToMeters(469.11));
  }
}
