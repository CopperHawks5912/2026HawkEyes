// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.fuel;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CANConstants;
import frc.robot.util.Utils;

public class FuelSubsystem extends SubsystemBase {
  // Hardware
  private final SparkMax leftIntakeLauncherMotor;
  private final SparkMax rightIntakeLauncherMotor;
  private final SparkMax feederMotor;

  // Shooter velocity control
  private SparkClosedLoopController leftController;
  private SparkClosedLoopController rightController;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;

  // Shooter state
  private double targetRPM = 0;
  
  // NetworkTables for tuning (works with Elastic Dashboard)
  private NetworkTable tuningTable;
  private NetworkTableEntry tuningDistanceEntry;
  private NetworkTableEntry tuningRPMEntry;
  private NetworkTableEntry currentDistanceEntry;
  private NetworkTableEntry currentRPMEntry;
  private NetworkTableEntry leftVelocityEntry;
  private NetworkTableEntry rightVelocityEntry;
  private NetworkTableEntry atSpeedEntry;
  
  // Mutable holders for unit-safe voltage values, persisted to avoid reallocation.
  private final InterpolatingDoubleTreeMap launcherRPM;
  private final MutVoltage appliedVoltage = Volts.mutable(0);
  private final MutAngle angle = Radians.mutable(0);
  private final MutAngularVelocity velocity = RadiansPerSecond.mutable(0);

  // Create a new SysId routine for characterizing the shooter.
  private final SysIdRoutine sysIdRoutine;
  
  /** Creates a new FuelSubsystem. */
  public FuelSubsystem() {
    // Initialize hardware (intake/launcher motors are NEOs, feeder is a brushed CIM motor)
    leftIntakeLauncherMotor = new SparkMax(CANConstants.kLeftIntakeLauncherMotorID, MotorType.kBrushless);
    rightIntakeLauncherMotor = new SparkMax(CANConstants.kRightIntakeLauncherMotorID, MotorType.kBrushless);
    feederMotor = new SparkMax(CANConstants.kFeederMotorID, MotorType.kBrushed);
    
    // Configure motors
    configureFeederMotor();
    configureIntakeLauncherMotors();

    // Initialize controllers
    leftController = leftIntakeLauncherMotor.getClosedLoopController();
    rightController = rightIntakeLauncherMotor.getClosedLoopController();

    // Initialize encoders
    leftEncoder = leftIntakeLauncherMotor.getEncoder();
    rightEncoder = rightIntakeLauncherMotor.getEncoder();

    // Initialize SysId routine for shooter characterization
    sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
        voltage -> {
          leftIntakeLauncherMotor.setVoltage(voltage);
          rightIntakeLauncherMotor.setVoltage(voltage);
        },
        log -> {
          log.motor("launcher-left")
            .voltage(appliedVoltage.mut_replace(leftIntakeLauncherMotor.get() * RobotController.getBatteryVoltage(), Volts))
            .angularPosition(angle.mut_replace(leftEncoder.getPosition(), Rotations))
            .angularVelocity(velocity.mut_replace(leftEncoder.getVelocity(), RotationsPerSecond));
          log.motor("launcher-right")
            .voltage(appliedVoltage.mut_replace(rightIntakeLauncherMotor.get() * RobotController.getBatteryVoltage(), Volts))
            .angularPosition(angle.mut_replace(rightEncoder.getPosition(), Rotations))
            .angularVelocity(velocity.mut_replace(rightEncoder.getVelocity(), RotationsPerSecond));
        },
        this
      )
    );
    
    // set the default command for this subsystem
    setDefaultCommand(stopCommand());

    // Initialize lookup table for shoot RPM based on distance
    // This will be populated from tuning data
    launcherRPM = new InterpolatingDoubleTreeMap();
    loadDefaultLauncherMap();

    // Initialize dashboard
    SmartDashboard.putData("Fuel", this);

    // Setup NetworkTables for tuning (works with Elastic)
    setupNetworkTables();
    
    // Output initialization progress
    Utils.logInfo("Fuel subsystem initialized");
  }
  
  /**
   * Load default launcher RPM map
   * These are starting values - tune them using Shuffleboard
   */
  private void loadDefaultLauncherMap() {
    // Clear existing map
    launcherRPM.clear();
    
    // Load from constants or use defaults
    launcherRPM.put(0.0, 1000.0);   // Close range
    launcherRPM.put(2.0, 2000.0);   // 2 meters
    launcherRPM.put(3.0, 2500.0);   // 3 meters
    launcherRPM.put(4.0, 3000.0);   // Mid range
    launcherRPM.put(5.0, 3500.0);   // 5 meters
    launcherRPM.put(6.0, 4000.0);   // 6 meters
    launcherRPM.put(7.0, 4500.0);   // Far range (max distance for half field)
  }
  
  /**
   * Setup NetworkTables for tuning the launcher
   * Compatible with both Shuffleboard and Elastic Dashboard
   */
  private void setupNetworkTables() {
    // Get or create the tuning table
    tuningTable = NetworkTableInstance.getDefault().getTable("LauncherTuning");
    
    // Create input entries with default values
    tuningDistanceEntry = tuningTable.getEntry("TuneDistance");
    tuningDistanceEntry.setDouble(5.0);
    
    tuningRPMEntry = tuningTable.getEntry("TuneRPM");
    tuningRPMEntry.setDouble(3500.0);
    
    // Create output/display entries
    currentDistanceEntry = tuningTable.getEntry("CurrentDistance");
    currentDistanceEntry.setDouble(0.0);
    
    currentRPMEntry = tuningTable.getEntry("CurrentTargetRPM");
    currentRPMEntry.setDouble(0.0);
    
    leftVelocityEntry = tuningTable.getEntry("LeftVelocity");
    leftVelocityEntry.setDouble(0.0);
    
    rightVelocityEntry = tuningTable.getEntry("RightVelocity");
    rightVelocityEntry.setDouble(0.0);
    
    atSpeedEntry = tuningTable.getEntry("AtSpeed");
    atSpeedEntry.setBoolean(false);
    
    // Add commands to SmartDashboard so they appear in Elastic
    SmartDashboard.putData("Launcher/TestTunedShot", testTunedShotCommand());
    SmartDashboard.putData("Launcher/SaveTuningPoint", saveTuningPointCommand());
    SmartDashboard.putData("Launcher/PrintLookupTable", printLookupTableCommand());
    SmartDashboard.putData("Launcher/ResetToDefaults", resetLookupTableCommand());
    
    Utils.logInfo("Launcher tuning NetworkTables initialized");
  }

  /**
   * Configure the feeder motor with all settings
   */
  private void configureFeederMotor() {
    SparkMaxConfig feederConfig = new SparkMaxConfig();

    // motor output
    feederConfig
      .smartCurrentLimit(40) // amps
      .voltageCompensation(12)
      .idleMode(IdleMode.kBrake);

    // Optimize CAN status frames for reduced lag
    feederConfig.signals
      .primaryEncoderPositionPeriodMs(500)  // Position: not used
      .primaryEncoderVelocityPeriodMs(20)   // Velocity: 50Hz (was Status2)
      .appliedOutputPeriodMs(100)           // Applied output: 10Hz (was Status0)
      .faultsPeriodMs(200)                  // Faults: 5Hz (was Status1)
      .analogVoltagePeriodMs(500)           // Analog: unused (was Status3)
      .externalOrAltEncoderPosition(500)    // Alt encoder: unused (was Status4)
      .externalOrAltEncoderVelocity(500);   // Alt encoder: unused (was Status4)

    // apply configuration
    feederMotor.configure(
      feederConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );
  }
  
  /**
   * Configure the intake/launcher motors with all settings
   */
  private void configureIntakeLauncherMotors() {
    SparkMaxConfig launcherConfig = new SparkMaxConfig();

    // configure intake/launcher motors
    launcherConfig
      .smartCurrentLimit(60) // amps
      .voltageCompensation(12)
      .idleMode(IdleMode.kCoast);

    // closed-loop velocity control parameters
    launcherConfig.closedLoop
      .outputRange(-1, 1)
      .p(FuelConstants.kLauncherP)
      .i(FuelConstants.kLauncherI)
      .d(FuelConstants.kLauncherD);

    // feedforward parameters
    launcherConfig.closedLoop.feedForward
      .kS(FuelConstants.kLauncherKS)
      .kV(FuelConstants.kLauncherKV)
      .kA(FuelConstants.kLauncherKA);

    // Optimize CAN status frames for reduced lag
    launcherConfig.signals
      .primaryEncoderPositionPeriodMs(500)  // Position: not used
      .primaryEncoderVelocityPeriodMs(20)   // Velocity: 50Hz (was Status2)
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

  @Override
  public void periodic() {
    // Update tuning table for Elastic
    leftVelocityEntry.setDouble(leftEncoder.getVelocity());
    rightVelocityEntry.setDouble(rightEncoder.getVelocity());
    atSpeedEntry.setBoolean(isAtSpeed());
  }
    
  // ==================== Internal State Modifiers ====================
  
  /**
   * Set launcher motors to a percentage of max power.
   * Used for non-PID controlled intake/eject operations.
   * @param power Percentage of voltage to apply (-1.0 to 1.0)
   */
  private void setLauncherPower(double power) {
    double clampedPower = MathUtil.clamp(power, -1, 1);
    targetRPM = 0; // Not using velocity control
    leftIntakeLauncherMotor.set(clampedPower);
    rightIntakeLauncherMotor.set(clampedPower);
  }
  
  /**
   * Set launcher motors to a specific velocity / RPM.
   * Used for PID controlled launching/shooting operations.
   * Launching should always be a positive RPM value.
   * @param rpm RPM to set the intake/launcher motors to
   */
  private void setLauncherRPM(double rpm) {
    // clamp RPM to valid range (NEO max ~6000 RPM) 
    double clampedRPM = MathUtil.clamp(rpm, 0, 6000); 

    // cache target RPM
    targetRPM = clampedRPM;
    
    // Use PID + feedforward for better tracking
    leftController.setSetpoint(clampedRPM, ControlType.kVelocity);
    rightController.setSetpoint(clampedRPM, ControlType.kVelocity);
  }
  
  /**
   * Set feeder motor to a percentage of max power.
   * @param power Percentage of voltage to apply (-1.0 to 1.0)
   */
  private void setFeederRoller(double power) {
    feederMotor.set(MathUtil.clamp(power, -1, 1));
  }
  
  /**
   * Check if launcher is at target speed
   * @return true if both motors are within tolerance of target RPM
   */
  private boolean isAtSpeed() {
    if (targetRPM == 0) {
      return false;
    }
  
    return MathUtil.isNear(targetRPM, leftEncoder.getVelocity(), FuelConstants.kLauncherToleranceRPM) && 
           MathUtil.isNear(targetRPM, rightEncoder.getVelocity(), FuelConstants.kLauncherToleranceRPM);
  }
  
  // ==================== Tuning Commands ====================
  
  /**
   * Command to test the currently tuned shot parameters
   * Uses values from Shuffleboard sliders
   */
  public Command testTunedShotCommand() {
    return run(() -> {
      double testDistance = tuningDistanceEntry.getDouble(5.0);
      double testRPM = tuningRPMEntry.getDouble(3500.0);
      
      // Update display
      currentDistanceEntry.setDouble(testDistance);
      currentRPMEntry.setDouble(testRPM);
      
      // Spin up to test RPM
      setLauncherRPM(testRPM);
      
      // Feed when at speed
      if (isAtSpeed()) {
        setFeederRoller(FuelConstants.kFeederLaunchingPercent);
      } else {
        setFeederRoller(FuelConstants.kFeederSpinUpPreLaunchPercent);
      }
    }).withName("TestTunedShot");
  }
  
  /**
   * Command to save the current tuning point to the lookup table
   * Saves the distance/RPM pair from Shuffleboard sliders
   */
  public Command saveTuningPointCommand() {
    return runOnce(() -> {
      double distance = tuningDistanceEntry.getDouble(5.0);
      double rpm = tuningRPMEntry.getDouble(3500.0);
      
      // Add or update the point in the map
      launcherRPM.put(distance, rpm);
      
      Utils.logInfo(String.format("Saved tuning point: %.1fm → %.0f RPM", distance, rpm));
      System.out.println(String.format("launcherRPM.put(%.1f, %.1f);", distance, rpm));
    }).withName("SaveTuningPoint");
  }
  
  /**
   * Command to print the entire lookup table to the console
   * Use this to copy/paste values into your code
   */
  public Command printLookupTableCommand() {
    return runOnce(() -> {
      System.out.println("========== LAUNCHER RPM LOOKUP TABLE ==========");
      System.out.println("Copy these lines into loadDefaultLauncherMap method:");
      System.out.println();
      
      // The TreeMap doesn't expose its entries directly, so we'll print what we know
      System.out.println("// Tuned launcher RPM values");
      System.out.println("launcherRPM.clear();");
      
      // Print sample distances and their interpolated values
      for (double dist = 0.0; dist <= 10.0; dist += 0.5) {
        double rpm = launcherRPM.get(dist);
        System.out.println(String.format("launcherRPM.put(%.1f, %.1f);", dist, rpm));
      }
      
      System.out.println();
      System.out.println("========== END LOOKUP TABLE ==========");
    }).withName("PrintLookupTable");
  }
  
  /**
   * Command to reset the lookup table to default values
   */
  public Command resetLookupTableCommand() {
    return runOnce(() -> {
      loadDefaultLauncherMap();
      Utils.logInfo("Reset launcher lookup table to defaults");
    }).withName("ResetLookupTable");
  }

  // ==================== Command Factories ====================
  
  /**
   * Command to stop the rollers
   * @return Command that stops the roller motors
   */
  public Command stopCommand() {
    return runOnce(() -> {
      setLauncherPower(0);
      setFeederRoller(0);
      targetRPM = 0;
    }).withName("StopIntake");
  }
  
  /**
   * Command to intake fuel from the ground
   * @return Command that runs roller at intake speed for fuel
   */
  public Command intakeCommand() {
    return run(() -> {
      setLauncherPower(FuelConstants.kIntakeIntakingPercent);
      setFeederRoller(FuelConstants.kFeederIntakingPercent);
    }).withName("IntakeFuel");
  }
  
  /**
   * Command to eject fuel out of the ground intake
   * @return Command that runs rollers at eject speed for fuel
   */
  public Command ejectCommand() {
    return run(() -> {
      setLauncherPower(FuelConstants.kIntakeEjectPercent);
      setFeederRoller(FuelConstants.kFeederEjectPercent);
    }).withName("EjectFuel");
  }
  
  /**
   * Command to pass fuel out of the launcher at a lower speed
   * Hold button: spins up → automatically feeds when ready
   * Release button: stops everything immediately
   * @return Command that runs rollers at passing speed
   */
  public Command passCommand() {
    return run(() -> {
      // Spin up to passing speed
      setLauncherRPM(FuelConstants.kLauncherPassingRPM);
      
      // Only feed when at speed - otherwise hold fuel back
      if (isAtSpeed()) {
        setFeederRoller(FuelConstants.kFeederPassingPercent);
      } else {
        setFeederRoller(FuelConstants.kFeederSpinUpPreLaunchPercent);
      }
    }).withName("PassFuel");
  }
  
  /**
   * Command to launch/shoot fuel with distance-based RPM
   * Hold button: spins up → automatically feeds when ready
   * Release button: stops everything immediately
   * @param distanceToHub Supplier that provides the distance to the hub in meters
   * @return Command that intelligently spins up then launches
   */
  public Command launchCommand(DoubleSupplier distanceToHub) {
    return run(() -> {
      // get the distance value
      double distance = distanceToHub.getAsDouble();

      // calculate target RPM based on distance
      double rpm = (distance >= 0.0 && distance <= 10.0)
        ? launcherRPM.get(distance) 
        : FuelConstants.kLauncherLaunchingRPM;
      
      // spin up the launcher
      setLauncherRPM(rpm);
      
      // only feed when at speed - otherwise hold fuel back
      if (isAtSpeed()) {
        setFeederRoller(FuelConstants.kFeederLaunchingPercent);
      } else {
        setFeederRoller(FuelConstants.kFeederSpinUpPreLaunchPercent);
      }
    }).withName("LaunchFuel");
  }
  
  /**
   * Command to launch at a fixed RPM
   * @return Command that launches fuel at default RPM
   */
  public Command launchCommand() {
    return launchCommand(() -> 4.13); // Default to mid-range distance
  }

  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamicCommand(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }
  
  // ==================== Telemetry Methods ====================

  /**
   * Get the average velocity of both launcher motors
   * @return Average velocity in RPM
   */
  public double getVelocityRPM() {
    return (leftEncoder.getVelocity() + rightEncoder.getVelocity()) / 2.0;
  }
  
  /**
   * Get the average current draw of both launcher motors
   * @return Average current in amps
   */
  public double getCurrent() {
    return (leftIntakeLauncherMotor.getOutputCurrent() + 
            rightIntakeLauncherMotor.getOutputCurrent()) / 2.0;
  }
  
  /**
   * Get the average temperature of both launcher motors
   * @return Average temperature in Celsius
   */
  public double getTemperature() {
    return (leftIntakeLauncherMotor.getMotorTemperature() + 
            rightIntakeLauncherMotor.getMotorTemperature()) / 2.0;
  }

  /**
   * Initialize Sendable for SmartDashboard
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("FuelSubsystem");
    builder.addDoubleProperty("Left RPM", () -> Utils.showDouble(leftEncoder.getVelocity()), null);
    builder.addDoubleProperty("Right RPM", () -> Utils.showDouble(rightEncoder.getVelocity()), null);
    builder.addDoubleProperty("Velocity (RPM)", () -> Utils.showDouble(getVelocityRPM()), null);
    builder.addDoubleProperty("Target RPM", () -> Utils.showDouble(targetRPM), null);
    builder.addDoubleProperty("Current (A)", () -> Utils.showDouble(getCurrent()), null);
    builder.addDoubleProperty("Temperature (C)", () -> Utils.showDouble(getTemperature()), null);
    builder.addBooleanProperty("At Speed", this::isAtSpeed, null);
  }
}
