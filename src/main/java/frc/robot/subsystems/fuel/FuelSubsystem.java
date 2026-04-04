// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.fuel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANConstants;
import frc.robot.util.Utils;

public class FuelSubsystem extends SubsystemBase {
  // Hardware (motors, encoders, etc.)
  private final TalonFX leftIntakeLauncherMotor;
  private final TalonFX rightIntakeLauncherMotor;
  private final TalonFX feederMotor;

  private final VelocityVoltage launcherVelocityRequest;
  private final VelocityVoltage feederVelocityRequest;
  
  /** Creates a new FuelSubsystem. */
  public FuelSubsystem() {
    // Initialize hardware (intake/launcher motors are Kraken x60s, feeder is a Falcon 500)
    leftIntakeLauncherMotor = new TalonFX(CANConstants.kLeftIntakeLauncherMotorID);
    rightIntakeLauncherMotor = new TalonFX(CANConstants.kRightIntakeLauncherMotorID);
    feederMotor = new TalonFX(CANConstants.kFeederMotorID);

    // Initialize the launcher and feeder velocity requests
    launcherVelocityRequest = new VelocityVoltage(0).withSlot(0);
    feederVelocityRequest = new VelocityVoltage(0).withSlot(0);

    // Configure motors
    configureIntakeLauncherMotors();
    configureFeederMotor();

    // set the default command for this subsystem
    setDefaultCommand(stopCommand());

    // Initialize dashboard
    SmartDashboard.putData("Fuel", this);
    
    // Output initialization progress
    Utils.logInfo("Fuel subsystem initialized");
  }
  
  /**
   * Configure the intake/launcher motors with all settings
   */
  private void configureIntakeLauncherMotors() {
    // Create a shared configuration object for both launcher/intake motors
    TalonFXConfiguration launcherConfig = new TalonFXConfiguration();

    // -------------------------------------------------------------------------
    // Shared configuration for both motors (inverted separately below)
    // -------------------------------------------------------------------------
    // Motor outputs
    launcherConfig.MotorOutput
      .withDutyCycleNeutralDeadband(0.001)  // 0.1% deadband (tight control)
      .withNeutralMode(NeutralModeValue.Coast)
      .withInverted(InvertedValue.CounterClockwise_Positive);
    
    // Current limits (hardcoded here for safety)
    launcherConfig.CurrentLimits
      .withSupplyCurrentLimitEnable(true)   // Enable supply limits
      .withSupplyCurrentLimit(30)                 // Peak current spike limit in Amps
      .withSupplyCurrentLowerLimit(25)       // Continuous current limit in Amps
      .withSupplyCurrentLowerTime(0.5)        // Time until lower current in seconds
      .withStatorCurrentLimitEnable(true)   // Enable stator limits
      .withStatorCurrentLimit(80);                // Max stator current in Amps (prevents overheating)

    // Voltage compensation
    launcherConfig.Voltage
      .withPeakForwardVoltage(12)                   // Max voltage when running motor forward
      .withPeakReverseVoltage(-12)                                        // Max voltage when running motor in reverse
      .withSupplyVoltageTimeConstant(0.02);  // Voltage filter time constant in seconds

    // Velocity PID (runs on onboard motor controller, tunable in constants)
    launcherConfig.Slot0
      .withKP(FuelConstants.kLauncherP)      // Proportional gain
      .withKI(FuelConstants.kLauncherI)      // Integral gain
      .withKD(FuelConstants.kLauncherD)      // Derivative gain
      .withKS(FuelConstants.kLauncherS)      // Static feedforward
      .withKV(FuelConstants.kLauncherV)      // Velocity feedforward
      .withKA(FuelConstants.kLauncherA);     // Acceleration feedforward
    
    // -------------------------------------------------------------------------
    // Apply the configuration to the RIGHT motor first (non-inverted)
    // -------------------------------------------------------------------------
    rightIntakeLauncherMotor.getConfigurator().apply(launcherConfig);

    // -------------------------------------------------------------------------
    // Invert the configuration for the LEFT motor (physically mirrored)
    // -------------------------------------------------------------------------
    launcherConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftIntakeLauncherMotor.getConfigurator().apply(launcherConfig);

    // -------------------------------------------------------
    // OPTIMIZE CAN STATUS FRAMES for reduced lag
    // -------------------------------------------------------
    rightIntakeLauncherMotor.getVelocity().setUpdateFrequency(50.0);      // velocity feedback
    rightIntakeLauncherMotor.getPosition().setUpdateFrequency(2.0);       // not used
    rightIntakeLauncherMotor.getMotorVoltage().setUpdateFrequency(25.0);  // motor voltage
    rightIntakeLauncherMotor.getSupplyCurrent().setUpdateFrequency(25.0); // supply current
    rightIntakeLauncherMotor.getTorqueCurrent().setUpdateFrequency(4.0);  // stator/torque current
    rightIntakeLauncherMotor.getDeviceTemp().setUpdateFrequency(4.0);     // temperature
    rightIntakeLauncherMotor.optimizeBusUtilization();

    leftIntakeLauncherMotor.getVelocity().setUpdateFrequency(50.0);      // velocity feedback
    leftIntakeLauncherMotor.getPosition().setUpdateFrequency(2.0);       // not used
    leftIntakeLauncherMotor.getMotorVoltage().setUpdateFrequency(25.0);  // motor voltage
    leftIntakeLauncherMotor.getSupplyCurrent().setUpdateFrequency(25.0); // supply current
    leftIntakeLauncherMotor.getTorqueCurrent().setUpdateFrequency(4.0);  // stator/torque current
    leftIntakeLauncherMotor.getDeviceTemp().setUpdateFrequency(4.0);     // temperature
    leftIntakeLauncherMotor.optimizeBusUtilization();
  }

  /**
   * Configure the feeder motor with all settings
   */
  private void configureFeederMotor() {
    // Create a configuration object for the feeder motor
    TalonFXConfiguration feederConfig = new TalonFXConfiguration();

    // -------------------------------------------------------------------------
    // Shared configuration for both motors (inverted separately below)
    // -------------------------------------------------------------------------
    // Motor outputs
    feederConfig.MotorOutput
      .withDutyCycleNeutralDeadband(0.001)  // 0.1% deadband (tight control)
      .withNeutralMode(NeutralModeValue.Coast)
      .withInverted(InvertedValue.CounterClockwise_Positive);
    
    // Current limits (hardcoded here for safety)
    feederConfig.CurrentLimits
      .withSupplyCurrentLimitEnable(true)   // Enable supply limits
      .withSupplyCurrentLimit(30)                 // Peak current spike limit in Amps
      .withSupplyCurrentLowerLimit(25)       // Continuous current limit in Amps
      .withSupplyCurrentLowerTime(0.5)        // Time until lower current in seconds
      .withStatorCurrentLimitEnable(true)   // Enable stator limits
      .withStatorCurrentLimit(80);                // Max stator current in Amps (prevents overheating)

    // Voltage compensation
    feederConfig.Voltage
      .withPeakForwardVoltage(12)                  // Max voltage when running motor forward
      .withPeakReverseVoltage(-12)                                       // Max voltage when running motor in reverse
      .withSupplyVoltageTimeConstant(0.02); // Voltage filter time constant in seconds

    // Velocity PID (runs on onboard motor controller, tunable in constants)
    feederConfig.Slot0
      .withKP(FuelConstants.kFeederP)      // Proportional gain
      .withKI(FuelConstants.kFeederI)      // Integral gain
      .withKD(FuelConstants.kFeederD)      // Derivative gain
      .withKS(FuelConstants.kFeederS)      // Static feedforward
      .withKV(FuelConstants.kFeederV)      // Velocity feedforward
      .withKA(FuelConstants.kFeederA);     // Acceleration feedforward
    
    // -------------------------------------------------------------------------
    // Apply the configuration to the RIGHT motor first (non-inverted)
    // -------------------------------------------------------------------------
    feederMotor.getConfigurator().apply(feederConfig);

    // -------------------------------------------------------
    // OPTIMIZE CAN STATUS FRAMES for reduced lag
    // -------------------------------------------------------
    feederMotor.getVelocity().setUpdateFrequency(50.0);      // velocity feedback
    feederMotor.getPosition().setUpdateFrequency(2.0);       // not used
    feederMotor.getMotorVoltage().setUpdateFrequency(25.0);  // motor voltage
    feederMotor.getSupplyCurrent().setUpdateFrequency(25.0); // supply current
    feederMotor.getTorqueCurrent().setUpdateFrequency(4.0);  // stator/torque current
    feederMotor.getDeviceTemp().setUpdateFrequency(4.0);     // temperature
    feederMotor.optimizeBusUtilization();
  }

  @Override
  public void periodic() {}
    
  // ==================== Internal State Modifiers ====================
  
  /**
   * Set launcher motors to a percentage of max power.
   * Used for non-PID controlled intake/eject operations.
   * @param power Percentage of voltage to apply (-1.0 to 1.0)
   */
  private void setLauncherPower(double power) {
    double clampedPower = MathUtil.clamp(power, -1, 1);
    rightIntakeLauncherMotor.set(clampedPower);
    leftIntakeLauncherMotor.set(clampedPower);
  }

  /**
   * Set feeder motor to a percentage of max power.
   * @param power Percentage of voltage to apply (-1.0 to 1.0)
   */
  private void setFeederPower(double power) {
    feederMotor.set(MathUtil.clamp(power, -1, 1));
  }
  
  /**
   * Set launcher motors to a desired RPS.
   * Used for PID controlled intake/eject operations.
   * @param rps Desired RPS for the launcher/intake motors
   */
  private void setLauncherRPS(double rps) {
    double clampedRPS = MathUtil.clamp(rps, -100, 100); // Kraken x60 max RPM is 6000, which is 100 RPS
    rightIntakeLauncherMotor.setControl(launcherVelocityRequest.withVelocity(clampedRPS));
    leftIntakeLauncherMotor.setControl(launcherVelocityRequest.withVelocity(clampedRPS));
  }

  /**
   * Set feeder motors to a desired RPS.
   * Used for PID controlled feed/intake/eject operations.
   * @param rps Desired RPS for the feeder motor
   */
  private void setFeederRPS(double rps) {
    double clampedRPS = MathUtil.clamp(rps, -106.333, 106.333); // Falcon 500 max RPM is 6380, which is 106.333 RPS
    feederMotor.setControl(feederVelocityRequest.withVelocity(clampedRPS));
  }
  
  /**
   * Stop all motors immediately
   */
  public void stop() {
    rightIntakeLauncherMotor.stopMotor();
    leftIntakeLauncherMotor.stopMotor();
    feederMotor.stopMotor();
  }

  // ==================== Command Factories ====================
  
  /**
   * Command to stop the all the rollers in the fuel subsystem immediately
   * @return Command that stops the roller motors
   */
  public Command stopCommand() {
    return run(this::stop)
      .withName("StopFuel");
  }
  
  /**
   * Command to intake fuel from the ground
   * @return Command that runs roller at intake speed for fuel
   */
  public Command intakeCommand() {
    return run(() -> {
      setLauncherPower(FuelConstants.kLauncherIntakingPercent);
      setFeederPower(FuelConstants.kFeederIntakingPercent);
    })
    .withName("IntakeFuel");
  }
  
  /**
   * Command to eject fuel out of the ground intake
   * @return Command that runs rollers at eject speed for fuel
   */
  public Command ejectCommand() {
    return run(() -> {
      setLauncherPower(FuelConstants.kLauncherEjectingPercent);
      setFeederPower(FuelConstants.kFeederEjectingPercent);
    })
    .withName("EjectFuel");
  }
  
  /**
   * Command to launch fuel out of the launcher at a fixed motor percentage.
   * Hold button: spins up for a brief period then feeds & launches.
   * Release button: stops everything immediately.
   * @return Command that runs rollers at launching power percentage
   */
  public Command launchPowerCommand() {
    return run(() -> {
      setLauncherPower(FuelConstants.kLauncherLaunchingPercent);
      setFeederPower(FuelConstants.kFeederSpinUpPreLaunchPercent);
    })
    .withTimeout(FuelConstants.kLauncherSpinUpTimeoutSeconds)
    .andThen(run(() -> {
      setLauncherPower(FuelConstants.kLauncherLaunchingPercent);
      setFeederPower(FuelConstants.kFeederLaunchingPercent);
    }))
    .withName("LaunchPowerFuel");
  }
  
  /**
   * Command to launch fuel out of the launcher at a fixed RPM.
   * Hold button: spins up for a brief period then feeds & launches.
   * Release button: stops everything immediately.
   * @return Command that runs rollers at launching RPS
   */
  public Command launchRpsCommand() {
    return run(() -> {
      setLauncherRPS(FuelConstants.kLauncherLaunchingRPS);
      setFeederRPS(FuelConstants.kFeederSpinUpPreLaunchRPS);
    })
    .withTimeout(FuelConstants.kLauncherSpinUpTimeoutSeconds)
    .andThen(run(() -> {
      setLauncherRPS(FuelConstants.kLauncherLaunchingRPS);
      setFeederRPS(FuelConstants.kFeederLaunchingRPS);
    }))
    .withName("LaunchRpsFuel");
  }
  
  // ==================== Telemetry Methods ====================

  /**
   * Get the average velocity of both launcher motors
   * @return Average velocity of the launcher motors in RPM
   */
  private double getVelocity() {
    return (leftIntakeLauncherMotor.getVelocity().getValueAsDouble() + 
            rightIntakeLauncherMotor.getVelocity().getValueAsDouble()) / 2.0 * 60.0;
  }

  /**
   * Get the average voltage applied to both launcher motors
   * @return Average voltage applied to the launcher motors
   */
  private double getVoltage() {
    return (leftIntakeLauncherMotor.getMotorVoltage().getValueAsDouble() + 
            rightIntakeLauncherMotor.getMotorVoltage().getValueAsDouble()) / 2.0;
  }
  
  /**
   * Get the average current draw of both launcher motors
   * @return Average current in amps
   */
  private double getCurrent() {
    return (leftIntakeLauncherMotor.getSupplyCurrent().getValueAsDouble() + 
            rightIntakeLauncherMotor.getSupplyCurrent().getValueAsDouble()) / 2.0;
  }
  
  /**
   * Get the average temperature of both launcher motors
   * @return Average temperature in Celsius
   */
  private double getTemperature() {
    return (leftIntakeLauncherMotor.getDeviceTemp().getValueAsDouble() + 
            rightIntakeLauncherMotor.getDeviceTemp().getValueAsDouble()) / 2.0;
  }

  /**
   * Initialize Sendable for SmartDashboard
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("FuelSubsystem");
    builder.addDoubleProperty("Velocity (rpm)", () -> Utils.showDouble(getVelocity()), null);
    builder.addDoubleProperty("Voltage (V)", () -> Utils.showDouble(getVoltage()), null);
    builder.addDoubleProperty("Current (A)", () -> Utils.showDouble(getCurrent()), null);
    builder.addDoubleProperty("Temperature (C)", () -> Utils.showDouble(getTemperature()), null);

    // right motor stats
    builder.addDoubleProperty("Right Velocity (rpm)", () -> Utils.showDouble(rightIntakeLauncherMotor.getVelocity().getValueAsDouble()), null);
    builder.addDoubleProperty("Right Voltage (V)", () -> Utils.showDouble(rightIntakeLauncherMotor.getMotorVoltage().getValueAsDouble()), null);
    builder.addDoubleProperty("Right Current (A)", () -> Utils.showDouble(rightIntakeLauncherMotor.getSupplyCurrent().getValueAsDouble()), null);
    builder.addDoubleProperty("Right Temperature (C)", () -> Utils.showDouble(rightIntakeLauncherMotor.getDeviceTemp().getValueAsDouble()), null);

    // left motor stats
    builder.addDoubleProperty("Left Velocity (rpm)", () -> Utils.showDouble(leftIntakeLauncherMotor.getVelocity().getValueAsDouble()), null);
    builder.addDoubleProperty("Left Voltage (V)", () -> Utils.showDouble(leftIntakeLauncherMotor.getMotorVoltage().getValueAsDouble()), null);
    builder.addDoubleProperty("Left Current (A)", () -> Utils.showDouble(leftIntakeLauncherMotor.getSupplyCurrent().getValueAsDouble()), null);
    builder.addDoubleProperty("Left Temperature (C)", () -> Utils.showDouble(leftIntakeLauncherMotor.getDeviceTemp().getValueAsDouble()), null);

    // feeder motor stats
    builder.addDoubleProperty("Feeder Velocity (rpm)", () -> Utils.showDouble(feederMotor.getVelocity().getValueAsDouble()), null);
    builder.addDoubleProperty("Feeder Voltage (V)", () -> Utils.showDouble(feederMotor.getMotorVoltage().getValueAsDouble()), null);
    builder.addDoubleProperty("Feeder Current (A)", () -> Utils.showDouble(feederMotor.getSupplyCurrent().getValueAsDouble()), null);
    builder.addDoubleProperty("Feeder Temperature (C)", () -> Utils.showDouble(feederMotor.getDeviceTemp().getValueAsDouble()), null);
  }
}
