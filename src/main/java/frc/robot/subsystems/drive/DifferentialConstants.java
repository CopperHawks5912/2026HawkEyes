// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.config.ModuleConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * Constants for the differential drive subsystem
 */
public final class DifferentialConstants {
  // ==================== Drive Control Parameters ====================

  /**
   * Translation scaling factor (0.0 to 1.0) - used by driveArcadeCommand
   * Use this to set the robot's max forward/backward speed when 
   * controlling with joysticks for safety or driver preference.
   */
  public static final double kMaxTranslationalSpeed = 0.8;
  
  /**
   * Rotation scaling factor (0.0 to 1.0) - used by driveArcadeCommand
   * Use this to set the robot's max rotation speed when controlling 
   * with joysticks for safety or driver preference.
   */
  public static final double kMaxRotationalSpeed = 0.6;
  
  /**
   * The slow mode speed (0.0 to 1.0) - used by driveArcadeCommand
   * This must be greater than the joystick deadband but less
   * than the max speed and rotation speeds set above.
   */
  public static final double kMaxSlowModeSpeed = 0.3;
  
  /**
   * Maximum speed in meters per second - used by Pathplanner
   * Typical values: 2-4 m/s for FRC drivetrains
   */
  public static final double kMaxSpeedMetersPerSecond = 4.0;
  
  /**
   * Maximum acceleration in meters per second squared - used by Pathplanner
   */
  public static final double kMaxAccelMetersPerSecondSq = 2.0;
  
  /**
   * Maximum angular speed (rotation) in radians per second - used by Pathplanner
   */
  public static final double kMaxAngularSpeedRadsPerSecond = 2.0 * Math.PI;
  
  /**
   * Maximum angular acceleration (rotation) in radians per second squared - used by Pathplanner
   */
  public static final double kMaxAngularAccelRadsPerSecondSq = Math.PI;
  
  // ==================== Joystick Control Parameters ====================
  
  /**
   * Joystick deadband (ignore inputs below this threshold)
   */
  public static final double kJoystickDeadband = 0.1;
  
  /**
   * Translational slew rate limit (units per second)
   * Limits how quickly forward/backward speed can change
   */
  public static final double kTranslationalSlewRateLimit = 3.0;
  
  /**
   * Rotational slew rate limit (units per second)
   * Limits how quickly rotation speed can change
   */
  public static final double kRotationalSlewRateLimit = 3.0;

  // ==================== Physical Robot Parameters ====================
  
  /**
   * Track width (distance between left and right wheels) in meters
   * Measure from the center of one wheel to the center of the other
   */
  public static final double kTrackWidthMeters = Units.inchesToMeters(21.625); 

  /**
   * Track length (distance between center of front wheel to back wheel) in meters
   * Measure from the center of one wheel to the center of the other
   */
  public static final double kTrackLengthMeters = Units.inchesToMeters(18);
  
  /**
   * The robot's mass in kilograms with the bumpers attached and battery installed
   * 2026 Rebuilt max 52.16 kg / 115 lbs without the battery or bumpers
   */
  public static final double kWheelCOF = 1.19;
  
  /**
   * The robot's mass in kilograms with the bumpers attached and battery installed
   * 2026 Rebuilt max 52.16 kg / 115 lbs without the battery or bumpers
   */
  public static final double kRobotMassKg = Units.lbsToKilograms(110.0);

  /**
   * Calculate the robot's moment of inertia - MOI (kg*m²)
   * MOI = 1/12 * mass * (lengthSquared + widthSquared)
   */
  public static final double kRobotMOI = 1/12 * kRobotMassKg * (Math.pow(kTrackLengthMeters, 2) + Math.pow(kTrackWidthMeters, 2));
  
  /**
   * Wheel diameter in meters
   */
  public static final double kWheelDiameterMeters = Units.inchesToMeters(6);
  
  /**
   * Wheel circumference in meters
   */
  public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;  
  
  /**
   * Rev through bore encoder v2 resolution (ticks per motor revolution)
   * See: https://revrobotics.ca/rev-11-3174/
   */
  public static final int kEncoderTicksPerRevolution = 8192;
  
  /**
   * Gear ratio from motor to wheel
   * If motor spins X times, wheel spins 1 time
   * Example: 10.71:1 gearbox means motor spins 10.71 times per wheel rotation
   */
  public static final double kGearRatio = 8.45;
  
  /**
   * Position conversion factor: converts encoder ticks to meters
   * Formula: wheel circumference / gear ratio
   */
  public static final double kPositionConversionFactor = kWheelCircumferenceMeters / kGearRatio;
  
  /**
   * Velocity conversion factor: converts encoder ticks/minute to meters/second
   * Formula: position conversion factor / 60 (to convert minutes to seconds)
   */
  public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0;
  
  // ==================== PID Constants for Velocity Control ====================
  
  /**
   * PID proportional gain for velocity control
   */
  public static final double kP = 1.5; // TODO: Tune this value

  /**
   * PID integral gain for velocity control
   */
  public static final double kI = 0.0; // TODO: Tune this value
  
  /**
   * PID derivative gain for velocity control
   */
  public static final double kD = 0.0; // TODO: Tune this value
  
  /**
   * Feedforward kS - voltage needed to overcome static friction
   * Units: Volts
   */
  public static final double kS = 0.22; // TODO: Use SysId to characterize
  
  /**
   * Feedforward kV - voltage needed per unit of velocity
   * Units: Volt-seconds/meter
   */
  public static final double kV = 2.6; // TODO: Use SysId to characterize
  
  /**
   * Feedforward kA - voltage needed per unit of acceleration
   * Units: Volt-seconds²/meter
   */
  public static final double kA = 0.4; // TODO: Use SysId to characterize
  
  // ==================== PID Constants for Aiming ====================
  
  /**
   * Aiming PID proportional gain
   */
  public static final double kAimP = 2.0; // TODO: Tune this value
  
  /**
   * Aiming PID integral gain
   */
  public static final double kAimI = 0.0;
  
  /**
   * Aiming PID derivative gain
   */
  public static final double kAimD = 0.1; // TODO: Tune this value
  
  /**
   * Aiming tolerance in radians (how close is "good enough")
   */
  public static final double kAimToleranceRad = Units.degreesToRadians(2.0);
  
  // ==================== Vision Parameters ====================
  
  /**
   * Maximum age of vision measurements to accept (seconds)
   */
  public static final double kVisionMeasurementMaxAge = 0.3;
  
  /**
   * Maximum allowed translation jump for vision measurements (meters)
   * Rejects vision measurements that are too far from current estimate
   */
  public static final double kVisionMaxTranslationJumpMeters = 1.0;
  
  /**
   * Maximum allowed rotation jump for vision measurements (degrees)
   * Rejects vision measurements with too much rotation difference
   */
  public static final double kVisionMaxRotationJumpDegrees = 30.0;
  
  // ==================== PathPlanner Configuration ====================
  
  /**
   * Robot configuration for PathPlanner
   * This is used by PathPlanner for trajectory generation
   */
  public static final RobotConfig kRobotConfig = new RobotConfig(
    kRobotMassKg,                   // mass (kg)
    kRobotMOI,                      // MOI (kg*m²)
    new ModuleConfig(
      kWheelDiameterMeters / 2.0,   // wheel radius (m)
      kMaxSpeedMetersPerSecond,     // max wheel speed (m/s)
      kWheelCOF,                    // wheel COF
      DCMotor.getCIM(2),  // drive motors: 2 CIMs per side but they act like 1 motor with 2x the torque and speed
      60.0,       // current limit (A)
      2                   // number of motors per module
    ),
    kTrackWidthMeters / 2.0         // module locations (just half track width for differential)
  );
}
