// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANConstants;
import frc.robot.util.Utils;

public class ClimberSubsystem extends SubsystemBase {
  // Hardware
  private final SparkMax climberMotor;
  private final RelativeEncoder climberEncoder;
  
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    // Initialize hardware (we're using a brushed CIM for the climber)
    climberMotor = new SparkMax(CANConstants.kClimberMotorID, MotorType.kBrushed);
    
    // Configure motor
    configureMotor();
    
    // Initialize encoder
    climberEncoder = climberMotor.getAlternateEncoder();
    
    // set the default command for this subsystem
    setDefaultCommand(stopCommand());

    // Initialize dashboard
    SmartDashboard.putData("Climber", this);
    
    // Output initialization progress
    Utils.logInfo("Climber subsystem initialized");
  }
  
  /**
   * Configure the climber motor with all settings
   */
  private void configureMotor() {
    SparkMaxConfig climbConfig = new SparkMaxConfig();

    // configure the climber motor
    climbConfig
      .smartCurrentLimit(40) // amps
      .voltageCompensation(12) // Consistent behavior across battery voltage
      .idleMode(IdleMode.kBrake); // CRITICAL: Brake mode prevents falling

    // configure the alternate encoder
    climbConfig.alternateEncoder
      .countsPerRevolution(ClimberConstants.kEncoderTicksPerRevolution) 
      .positionConversionFactor(ClimberConstants.kPositionConversionFactor)
      .velocityConversionFactor(ClimberConstants.kVelocityConversionFactor);

    // Optimize CAN status frames for reduced lag
    climbConfig.signals
      .externalOrAltEncoderPosition(40)      // Fast enough for limit detection
      .externalOrAltEncoderVelocity(500)     // Not used
      .primaryEncoderPositionPeriodMs(500)   // Not used (CIM has no built-in encoder)
      .primaryEncoderVelocityPeriodMs(500)   // Not used (CIM has no built-in encoder)
      .appliedOutputPeriodMs(500)            // Not needed for open-loop control
      .faultsPeriodMs(200)                   // Keep at 200ms for fault detection
      .analogVoltagePeriodMs(500);           // Not used

    // apply configuration
    climberMotor.configure(
      climbConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );
  }

  @Override
  public void periodic() {  
    // Check limits and stop if exceeded
    if (isAtUpperLimit() || isAtLowerLimit()) {
      setPower(0);
    }
  }
    
  // ==================== Internal State Modifiers ====================
  
  /**
   * Set climber motor power with safety limits
   * @param power Power to apply (-1.0 to 1.0)
   */
  private void setPower(double power) {
    double clampedPower = MathUtil.clamp(power, -1, 1);

    // Safety: Stop at limits to prevent damage
    // Use > instead of >= to create a "close to limit" zone
    if (getPosition() > ClimberConstants.kUpperLimitDegrees && clampedPower > 0) {
      climberMotor.set(0);
      return;
    }
    
    if (getPosition() < ClimberConstants.kLowerLimitDegrees && clampedPower < 0) {
      climberMotor.set(0);
      return;
    }
    
    // If within limits, set the motor power
    climberMotor.set(clampedPower);
  }
  
  /**
   * Reset the encoder position to zero
   */
  public void resetEncoder() {
    climberEncoder.setPosition(0);
  }
  
  // ==================== State Queries ====================
  
  /**
   * Get the current position of the climber
   * @return Position in degrees
   */
  public double getPosition() {
    return climberEncoder.getPosition();
  }
  
  /**
   * Get the current draw of the climber motor
   * @return Current in amps
   */
  public double getCurrent() {
    return climberMotor.getOutputCurrent();
  }
  
  /**
   * Get the temperature of the climber motor
   * @return Temperature in Celsius
   */
  public double getTemperature() {
    return climberMotor.getMotorTemperature();
  }
  
  /**
   * Check if climber is at upper soft limit
   * @return true if at or past upper limit
   */
  public boolean isAtUpperLimit() {
    return MathUtil.isNear(
      ClimberConstants.kUpperLimitDegrees,
      getPosition(),
      ClimberConstants.kPositionToleranceDegrees
    );
  }
  
  /**
   * Check if climber is at lower soft limit
   * @return true if at or past lower limit
   */
  public boolean isAtLowerLimit() {
    return MathUtil.isNear(
      ClimberConstants.kLowerLimitDegrees,
      getPosition(),
      ClimberConstants.kPositionToleranceDegrees
    );
  }
  
  /**
   * Check if climber is at home position
   * @return true if within tolerance of home position
   */
  public boolean isAtHomePosition() {
    return MathUtil.isNear(
      ClimberConstants.kHomeDegrees,
      getPosition(),
      ClimberConstants.kPositionToleranceDegrees
    );
  }
  
  /**
   * Check if climber is stalled (high current, low velocity)
   * Useful for detecting when climber hits a hard stop
   * @return true if motor appears stalled
   */
  public boolean isStalled() {
    return Math.abs(getCurrent()) > ClimberConstants.kStallCurrentThreshold &&
           Math.abs(climberEncoder.getVelocity()) < ClimberConstants.kStallVelocityThreshold;
  }
  
  // ==================== Command Factories ====================
  
  /**
   * Command to extend the climber upward
   * @return Command that runs climber up at configured speed
   */
  public Command upCommand() {
    return run(() -> setPower(ClimberConstants.kUpPercent))
      .withName("ClimberUp");
  }
  
  /**
   * Command to retract the climber downward
   * @return Command that runs climber down at configured speed
   */
  public Command downCommand() {
    return run(() -> setPower(ClimberConstants.kDownPercent))
      .withName("ClimberDown");
  }
  
  /**
   * Command the climber up until upper limit
   * @return Command that rotates to upper limit then stops
   */
  public Command upToLimitCommand() {
    return run(() -> setPower(ClimberConstants.kUpPercent))
      .until(this::isAtUpperLimit)
      .andThen(stopCommand())
      .withName("UpToLimit");
  }
  
  /**
   * Command the climber down until lower limit
   * @return Command that rotates to lower limit then stops
   */
  public Command downToLimitCommand() {
    return run(() -> setPower(ClimberConstants.kDownPercent))
      .until(this::isAtLowerLimit)
      .andThen(stopCommand())
      .withName("DownToLimit");
  }
  
  /**
   * Command the climber to move to the upright home position
   * @return Command that moves to the home position then stops
   */
  public Command homeCommand() {
    return run(() -> {
      if (getPosition() > ClimberConstants.kHomeDegrees) {
        setPower(ClimberConstants.kDownPercent);
      } else {
        setPower(ClimberConstants.kUpPercent);
      }
    })
    .until(this::isAtHomePosition)
    .andThen(stopCommand())
    .withName("HomeClimber");
  }
  
  /**
   * Command to stop the climber
   * @return Command that stops the climber motor
   */
  public Command stopCommand() {
    return runOnce(() -> setPower(0))
      .withName("StopClimber");
  }
  
  /**
   * Command to reset encoder to zero at current position
   * Use this when climber is at a known position (e.g., fully retracted)
   * @return Command that resets the encoder
   */
  public Command resetEncoderCommand() {
    return runOnce(this::resetEncoder)
      .withName("ResetClimberEncoder");
  }
  
  // ==================== Telemetry Methods ====================
  
  /**
   * Initialize Sendable for SmartDashboard
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("ClimberSubsystem");
    builder.addDoubleProperty("Position (degrees)", this::getPosition, null);
    builder.addDoubleProperty("Current (A)", this::getCurrent, null);
    builder.addDoubleProperty("Temperature (C)", this::getTemperature, null);
    builder.addBooleanProperty("At Upper Limit", this::isAtUpperLimit, null);
    builder.addBooleanProperty("At Lower Limit", this::isAtLowerLimit, null);
    builder.addBooleanProperty("At Home Position", this::isAtHomePosition, null);
    builder.addBooleanProperty("Stalled", this::isStalled, null);
  }
}
