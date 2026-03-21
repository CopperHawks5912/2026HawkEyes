// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.fuel;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANConstants;
import frc.robot.util.Utils;

public class FuelSubsystem extends SubsystemBase {
  // Hardware
  private final SparkMax leftIntakeLauncherMotor;
  private final SparkMax rightIntakeLauncherMotor;
  private final SparkMax feederMotor;
  private final RelativeEncoder leftIntakeLauncherEncoder;
  private final RelativeEncoder rightIntakeLauncherEncoder;
  
  // NetworkTables for tuning (works with Elastic Dashboard)
  private NetworkTable tuningTable;
  private NetworkTableEntry tuningDistanceEntry;
  private NetworkTableEntry tuningPowerEntry;
  private NetworkTableEntry currentDistanceEntry;
  private NetworkTableEntry currentPowerEntry;
  
  // Distance to power lookup table
  private final InterpolatingDoubleTreeMap launcherPower;
  
  /** Creates a new FuelSubsystem. */
  public FuelSubsystem() {
    // Initialize hardware (intake/launcher motors are NEOs, feeder is a brushed CIM motor)
    leftIntakeLauncherMotor = new SparkMax(CANConstants.kLeftIntakeLauncherMotorID, MotorType.kBrushless);
    rightIntakeLauncherMotor = new SparkMax(CANConstants.kRightIntakeLauncherMotorID, MotorType.kBrushless);
    feederMotor = new SparkMax(CANConstants.kFeederMotorID, MotorType.kBrushed);
    
    // Configure motors
    configureFeederMotor();
    configureIntakeLauncherMotors();

    // Get the NEO builtin encoders (after motor configuration)
    leftIntakeLauncherEncoder = leftIntakeLauncherMotor.getEncoder();
    rightIntakeLauncherEncoder = rightIntakeLauncherMotor.getEncoder();

    // set the default command for this subsystem
    setDefaultCommand(stopCommand());

    // Initialize lookup table for launching power based on distance
    launcherPower = new InterpolatingDoubleTreeMap();
    loadLauncherPowers();

    // Setup NetworkTables for tuning (works with Elastic)
    setupNetworkTables();

    // Initialize dashboard
    SmartDashboard.putData("Fuel", this);
    
    // Output initialization progress
    Utils.logInfo("Fuel subsystem initialized");
  }
  
  /**
   * Configure the intake/launcher motors with all settings
   */
  private void configureIntakeLauncherMotors() {
    SparkMaxConfig launcherConfig = new SparkMaxConfig();

    // configure intake/launcher motors
    launcherConfig
      .smartCurrentLimit(30) // amps
      .voltageCompensation(12)
      .idleMode(IdleMode.kCoast);

    // Optimize CAN status frames for reduced lag
    launcherConfig.signals
      .primaryEncoderPositionPeriodMs(500)  // Position: not used
      .primaryEncoderVelocityPeriodMs(500)  // Velocity: not used
      .appliedOutputPeriodMs(100)           // Applied output: 10Hz (was Status0)
      .faultsPeriodMs(200)                  // Faults: 5Hz (was Status1)
      .analogVoltagePeriodMs(500)           // Analog: unused (was Status3)
      .externalOrAltEncoderPosition(500)    // Alt encoder: unused (was Status4)
      .externalOrAltEncoderVelocity(500);   // Alt encoder: unused (was Status4)

    // configure the right motor
    rightIntakeLauncherMotor.configure(
      launcherConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );
    
    // invert the left motor
    launcherConfig.inverted(true);

    // configure the left motor
    leftIntakeLauncherMotor.configure(
      launcherConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );    
  }

  /**
   * Configure the feeder motor with all settings
   */
  private void configureFeederMotor() {
    SparkMaxConfig feederConfig = new SparkMaxConfig();

    // motor output
    feederConfig
      .smartCurrentLimit(30) // amps
      .voltageCompensation(12)
      .idleMode(IdleMode.kBrake);

    // Optimize CAN status frames for reduced lag
    feederConfig.signals
      .primaryEncoderPositionPeriodMs(500)  // Position: not used
      .primaryEncoderVelocityPeriodMs(500)  // Velocity: not used
      .appliedOutputPeriodMs(100)           // Applied output: 10Hz (was Status0)
      .faultsPeriodMs(200)                  // Faults: 5Hz (was Status1)
      .analogVoltagePeriodMs(500)           // Analog: not used
      .externalOrAltEncoderPosition(500)    // Alt encoder: not used
      .externalOrAltEncoderVelocity(500);   // Alt encoder: not used

    // apply configuration
    feederMotor.configure(
      feederConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );
  }

  /**
   * Load launcher distance to power map
   */
  private void loadLauncherPowers() {
    launcherPower.clear();
    launcherPower.put(0.0, 0.30);  // 0.0 meters - close range
    launcherPower.put(1.0, 0.40);  // 1.0 meters
    launcherPower.put(2.0, 0.50);  // 2.0 meters
    launcherPower.put(3.0, 0.60);  // 3.0 meters
    launcherPower.put(4.0, 0.70);  // 4.0 meters
    launcherPower.put(5.0, 0.80);  // 5.0 meters
    launcherPower.put(6.0, 0.90);  // 6.0 meters
  }

  /**
   * Setup NetworkTables for tuning the launcher
   */
  private void setupNetworkTables() {
    // Get or create the tuning table
    tuningTable = NetworkTableInstance.getDefault().getTable("LauncherTuning");
    
    // Create input entries with default values
    tuningDistanceEntry = tuningTable.getEntry("TuneDistance");
    tuningDistanceEntry.setDouble(5.0);
    
    tuningPowerEntry = tuningTable.getEntry("TunePower");
    tuningPowerEntry.setDouble(0.80);
    
    // Create output/display entries
    currentDistanceEntry = tuningTable.getEntry("CurrentDistance");
    currentDistanceEntry.setDouble(0.0);
    
    currentPowerEntry = tuningTable.getEntry("CurrentPower");
    currentPowerEntry.setDouble(0.0);
    
    // Add commands to SmartDashboard so they appear in Elastic
    SmartDashboard.putData("Launcher/TestTunedShot", testTunedShotCommand());
    SmartDashboard.putData("Launcher/ResetToDefaults", resetLookupTableCommand());
    
    Utils.logInfo("Launcher tuning NetworkTables initialized");
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
    // clamp power to valid range
    double clampedPower = MathUtil.clamp(power, -1, 1);

    // Set motor power directly (for open-loop control)
    leftIntakeLauncherMotor.set(clampedPower);
    rightIntakeLauncherMotor.set(clampedPower);
  }

  /**
   * Set feeder motor to a percentage of max power.
   * @param power Percentage of voltage to apply (-1.0 to 1.0)
   */
  private void setFeederPower(double power) {
    feederMotor.set(MathUtil.clamp(power, -1, 1));
  }
  
  /**
   * Stop all motors immediately
   */
  private void stop() {
    leftIntakeLauncherMotor.stopMotor();
    rightIntakeLauncherMotor.stopMotor();
    feederMotor.stopMotor();
  }

  // ==================== Tuning Commands ====================
  
  /**
   * Command to test the currently tuned shot parameters
   * Uses values from Shuffleboard sliders
   */
  public Command testTunedShotCommand() {
    return run(() -> {
      double power = MathUtil.clamp(tuningPowerEntry.getDouble(0.80), 0, 1);
      currentPowerEntry.setDouble(power);
      setLauncherPower(power);
      setFeederPower(FuelConstants.kFeederSpinUpPreLaunchPercent);
    })
    .withTimeout(FuelConstants.kLauncherSpinUpTimeoutSeconds)
    .andThen(run(() -> {
      double power = MathUtil.clamp(tuningPowerEntry.getDouble(0.80), 0, 1);
      currentPowerEntry.setDouble(power);
      setLauncherPower(power);
      setFeederPower(FuelConstants.kFeederLaunchingPercent);
    }))
    .withName("TestTunedShot");
  }
  
  /**
   * Command to reset the lookup table to default values
   */
  public Command resetLookupTableCommand() {
    return runOnce(() -> {
      loadLauncherPowers();
      Utils.logInfo("Reset launcher lookup table to defaults");
    }).withName("ResetLookupTable");
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
   * Command to pass fuel out of the launcher at a lower speed
   * Hold button: spins up → automatically feeds when ready
   * Release button: stops everything immediately
   * @return Command that runs rollers at passing speed
   */
  public Command passCommand() {
    return run(() -> {
      setLauncherPower(FuelConstants.kLauncherPassingPercent);
      setFeederPower(FuelConstants.kFeederSpinUpPreLaunchPercent);
    })
    .withTimeout(FuelConstants.kLauncherSpinUpTimeoutSeconds)
    .andThen(run(() -> {
      setLauncherPower(FuelConstants.kLauncherPassingPercent);
      setFeederPower(FuelConstants.kFeederPassingPercent);
    }))
    .withName("PassFuel");
  }
  
  /**
   * Command to launch fuel out of the launcher at a fixed speed and distance
   * Hold button: spins up for a brief period then feeds & launches
   * Release button: stops everything immediately
   * @return Command that runs rollers at launching speed
   */
  public Command launchCommand() {
    return run(() -> {
      setLauncherPower(FuelConstants.kLauncherLaunchingPercent);
      setFeederPower(FuelConstants.kFeederSpinUpPreLaunchPercent);
    })
    .withTimeout(FuelConstants.kLauncherSpinUpTimeoutSeconds)
    .andThen(run(() -> {
      setLauncherPower(FuelConstants.kLauncherLaunchingPercent);
      setFeederPower(FuelConstants.kFeederLaunchingPercent);
    }))
    .withName("LaunchFuel");
  }
  
  /**
   * Command to launch/shoot fuel based on how far a button is pressed
   * Hold button: spins up for a brief period then feeds & launches
   * Release button: stops everything immediately
   * @param powerSupplier Supplier that provides the launcher power based on button input (0.0 to 1.0)
   * @return Command that intelligently spins up then launches
   */
  public Command launchPowerCommand(DoubleSupplier powerSupplier) {
    return run(() -> {
      setLauncherPower(MathUtil.clamp(powerSupplier.getAsDouble(), 0.0, 1.0));
      setFeederPower(FuelConstants.kFeederSpinUpPreLaunchPercent);
    })
    .withTimeout(FuelConstants.kLauncherSpinUpTimeoutSeconds)
    .andThen(run(() -> {
      setLauncherPower(MathUtil.clamp(powerSupplier.getAsDouble(), 0.0, 1.0));
      setFeederPower(FuelConstants.kFeederLaunchingPercent);
    }))
    .withName("LaunchPowerFuel");
  }
  
  /**
   * Command to launch/shoot fuel with distance-based power using the lookup table
   * Hold button: spins up for a brief period then feeds & launches
   * Release button: stops everything immediately
   * @param distanceToHub Supplier that provides the distance to the hub in meters
   * @return Command that intelligently spins up then launches
   */
  public Command launchDistanceCommand(DoubleSupplier distanceToHub) {
    return run(() -> {
      setLauncherPower(launcherPower.get(MathUtil.clamp(distanceToHub.getAsDouble(), 0.0, 6.0)));
      setFeederPower(FuelConstants.kFeederSpinUpPreLaunchPercent);
    })
    .withTimeout(FuelConstants.kLauncherSpinUpTimeoutSeconds)
    .andThen(run(() -> {
      setLauncherPower(launcherPower.get(MathUtil.clamp(distanceToHub.getAsDouble(), 0.0, 6.0)));
      setFeederPower(FuelConstants.kFeederLaunchingPercent);
    }))
    .withName("LaunchDistanceFuel");
  }
  
  // ==================== Telemetry Methods ====================

  /**
   * Get the average velocity of both launcher motors
   * @return Average velocity of the launcher motors in native RPM
   */
  private double getVelocity() {
    return (leftIntakeLauncherEncoder.getVelocity() + 
            rightIntakeLauncherEncoder.getVelocity()) / 2.0;
  }

  /**
   * Get the average voltage applied to both launcher motors
   * @return Average voltage applied to the launcher motors
   */
  private double getVoltage() {
    return (leftIntakeLauncherMotor.getAppliedOutput() + 
            rightIntakeLauncherMotor.getAppliedOutput()) / 2.0;
  }
  
  /**
   * Get the average current draw of both launcher motors
   * @return Average current in amps
   */
  private double getCurrent() {
    return (leftIntakeLauncherMotor.getOutputCurrent() + 
            rightIntakeLauncherMotor.getOutputCurrent()) / 2.0;
  }
  
  /**
   * Get the average temperature of both launcher motors
   * @return Average temperature in Celsius
   */
  private double getTemperature() {
    return (leftIntakeLauncherMotor.getMotorTemperature() + 
            rightIntakeLauncherMotor.getMotorTemperature()) / 2.0;
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
  }
}
