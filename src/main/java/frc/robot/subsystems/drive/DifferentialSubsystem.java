// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Set;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.path.PathConstraints;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.studica.frc.Navx;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.util.Utils;

public class DifferentialSubsystem extends SubsystemBase {
  // Hardware
  private final SparkMax leftLeaderMotor;
  private final SparkMax leftFollowerMotor;
  private final SparkMax rightLeaderMotor;
  private final SparkMax rightFollowerMotor;
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;
  private final DifferentialDrive drive;
  private final Navx gyro;

  // Kinematics and pose estimation
  private final DifferentialDriveKinematics kinematics;
  private final DifferentialDrivePoseEstimator poseEstimator;

  // PID controllers for driving
  private final PIDController leftPIDController;
  private final PIDController rightPIDController;
  private final SimpleMotorFeedforward feedForward;

  // PID controller for aiming
  private final ProfiledPIDController aimPIDController;

  // Slew rate limiters to make joystick inputs smoother
  private final SlewRateLimiter xSpeedLimiter;
  private final SlewRateLimiter rSpeedLimiter;

  // Field visualization
  private final Field2d field2d = new Field2d();

  // Flag to indicate if drive controls are inverted (e.g. for climbing)
  private boolean inverted = false;
  private boolean slowMode = false;
  private int addedVisionMeasurementCount = 0;

  /**
   * Creates a new DifferentialSubsystem.
   */
  public DifferentialSubsystem() {
    // Initialize the slew rate limiters
    xSpeedLimiter = new SlewRateLimiter(DifferentialConstants.kTranslationalSlewRateLimit);
    rSpeedLimiter = new SlewRateLimiter(DifferentialConstants.kRotationalSlewRateLimit);

    // Create the gyro
    gyro = new Navx(CANConstants.kGyroID);

    // Disable certain gyro messages to optimize CAN bus saturation
    gyro.enableOptionalMessages(
      true,
      true,
      false,
      false,
      false,
      true,
      false,
      false,
      false,
      true
    );

    // Initialize drive motors with correct MotorType (we're using brushed CIM motors)
    leftLeaderMotor = new SparkMax(CANConstants.kLeftDifferentialLeaderMotorID, MotorType.kBrushed);
    leftFollowerMotor = new SparkMax(CANConstants.kLeftDifferentialFollowerMotorID, MotorType.kBrushed);
    rightLeaderMotor = new SparkMax(CANConstants.kRightDifferentialLeaderMotorID, MotorType.kBrushed);
    rightFollowerMotor = new SparkMax(CANConstants.kRightDifferentialFollowerMotorID, MotorType.kBrushed);

    // Configure motors (do this before creating DifferentialDrive and getting encoders)
    configureMotors();

    // Get the alternate encoders - Rev throughbore (after motor configuration)
    leftEncoder = leftLeaderMotor.getEncoder();
    rightEncoder = rightLeaderMotor.getEncoder();

    // set up differential drive class
    drive = new DifferentialDrive(leftLeaderMotor, rightLeaderMotor);

    // Initialize PID controllers for autonomous driving
    leftPIDController = new PIDController(DifferentialConstants.kP, DifferentialConstants.kI, DifferentialConstants.kD);
    rightPIDController = new PIDController(DifferentialConstants.kP, DifferentialConstants.kI, DifferentialConstants.kD);
    feedForward = new SimpleMotorFeedforward(
      DifferentialConstants.kS,
      DifferentialConstants.kV,
      DifferentialConstants.kA
    );

    // Initialize PID controller for auto aiming
    aimPIDController = new ProfiledPIDController(
      DifferentialConstants.kAimP,
      DifferentialConstants.kAimI, 
      DifferentialConstants.kAimD,
      new TrapezoidProfile.Constraints(
        DifferentialConstants.kMaxAngularSpeedRadsPerSecond,
        DifferentialConstants.kMaxAngularAccelRadsPerSecondSq
      )
    );

    // CRITICAL: Enable continuous input for angle wrapping
    aimPIDController.enableContinuousInput(-Math.PI, Math.PI);
    aimPIDController.setTolerance(DifferentialConstants.kAimToleranceRad);

    // set up kinematics
    kinematics = new DifferentialDriveKinematics(DifferentialConstants.kTrackWidthMeters);

    // set up pose estimator
    poseEstimator = new DifferentialDrivePoseEstimator(
      kinematics, 
      gyro.getRotation2d(), 
      0.0, 
      0.0, 
      new Pose2d(),
      VecBuilder.fill(0.02, 0.02, 0.01), // State standard deviations (x, y, theta)
      VecBuilder.fill(0.1, 0.1, 0.1)     // Vision standard deviations (will be overridden)
    );

    // Configure AutoBuilder for path following
    AutoBuilder.configure(
      this::getPose, // Robot pose supplier
      this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier
      (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ChassisSpeeds
      new PPLTVController(0.02), // 20ms periodic cycle
      DifferentialConstants.kRobotConfig, // Robot configuration
      Utils::isRedAlliance, // Method to flip path based on alliance color
      this // Reference to this subsystem to set requirements
    );

    // Reset odometry to starting pose
    resetOdometry();

    // set the default command for this subsystem
    setDefaultCommand(stopCommand());
    
    // Initialize field visualization
    field2d.setRobotPose(getPose());

    // Initialize dashboard
    SmartDashboard.putData("Drive/Field", field2d);
    SmartDashboard.putData("Drive/Gyro", builder -> {
      builder.setSmartDashboardType("Gyro");
      builder.addDoubleProperty("Value", () -> gyro.getYaw().in(Degrees), null);
    });
    SmartDashboard.putData("Drive/Differential", this);
    
    // Output initialization progress
    Utils.logInfo("Differential subsystem initialized");
  }
  
  /**
   * Configure the motors with all settings
   */
  private void configureMotors() {
    // Create a shared motor configuration instance
    SparkMaxConfig motorConfig = new SparkMaxConfig();

    // Create the configuration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different battery 
    // voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping breakers.
    motorConfig
      .smartCurrentLimit(30) // amps
      .voltageCompensation(12) // volts
      .idleMode(IdleMode.kBrake);

    // Set the position and velocity conversion factors for the encoders
    // This converts encoder ticks to meters and meters/second, which 
    // allows us to work in real-world units for control and odometry
    motorConfig.encoder
      .countsPerRevolution(DifferentialConstants.kEncoderTicksPerRevolution)
      .positionConversionFactor(DifferentialConstants.kPositionConversionFactor)
      .velocityConversionFactor(DifferentialConstants.kVelocityConversionFactor);

    // Optimize CAN status frames for reduced lag for followers
    // Followers can have slower updates since they are just mirroring 
    // the leaders, while leaders need faster updates for position and 
    // velocity for odometry and closed-loop control
    motorConfig.signals
      .primaryEncoderPositionPeriodMs(500)  // Not used on follower
      .primaryEncoderVelocityPeriodMs(500)  // Not used on follower
      .externalOrAltEncoderPosition(500)    // Not used on follower
      .externalOrAltEncoderVelocity(500)    // Not used on follower
      .appliedOutputPeriodMs(500)           // Not used on follower
      .faultsPeriodMs(500)                  // Not used on follower
      .analogVoltagePeriodMs(500);          // Not used on follower

    // Set configuration to follow each leader and then apply it to corresponding
    // follower. Resetting in case a new controller is swapped in and persisting 
    // in case of a controller reset due to breaker trip
    motorConfig.follow(leftLeaderMotor);
    leftFollowerMotor.configure(
      motorConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );
    
    motorConfig.follow(rightLeaderMotor);
    rightFollowerMotor.configure(
      motorConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );

    // Optimize CAN status frames for reduced lag for leaders
    // Leaders need faster updates for position and velocity for odometry and closed-loop control, 
    // while followers can be slower since they are just mirroring the leaders
    motorConfig.signals
      .primaryEncoderPositionPeriodMs(20)   // ensure motor is in ALTNERATE_ENCODER mode
      .primaryEncoderVelocityPeriodMs(20)   // ensure motor is in ALTNERATE_ENCODER mode
      .externalOrAltEncoderPosition(500)    // Not used
      .externalOrAltEncoderVelocity(500)    // Not used
      .appliedOutputPeriodMs(100)           // Applied output: 10Hz (was Status0)
      .faultsPeriodMs(200)                  // Faults: 5Hz (was Status1)
      .analogVoltagePeriodMs(500);          // Not used
    
    // Remove following, then apply config to right leader
    motorConfig.disableFollowerMode();
    rightLeaderMotor.configure(
      motorConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );
    
    // Set config to inverted and then apply to left leader. Set Left side inverted
    // so that positive values drive both sides forward
    motorConfig.inverted(true);
    leftLeaderMotor.configure(
      motorConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );
  }

  @Override
  public void periodic() {
    // CRITICAL - Feed the motor safety watchdog
    // drive.feed();

    // Update the pose estimator with the latest readings
    poseEstimator.updateWithTime(
      Timer.getFPGATimestamp(), 
      gyro.getRotation2d(), 
      leftEncoder.getPosition(), 
      rightEncoder.getPosition()
    );
    
    // Update field visualization
    field2d.setRobotPose(getPose());
  }

  // ==================== Vision Measurement Consumer ====================

  /**
   * Add a vision measurement to the pose estimator with validation and filtering.
   * This method can be called by the VisionSubsystem whenever a new vision 
   * measurement is available.
   * @param visionPose The vision pose to add
   * @param timestamp The timestamp of the vision measurement
   * @param stdDevs The standard deviations of the vision measurement
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp, double[] stdDevs) {
    // Get current time
    double now = Timer.getFPGATimestamp();

    try {      
      // Validate components
      if (visionPose == null || stdDevs == null || stdDevs.length < 3) {
        return;
      }

      // Reject timestamps older than 0.3 seconds
      if ((now - timestamp) > DifferentialConstants.kVisionMeasurementMaxAge) {
        return;
      }

      // Reject timestamps from the future
      if (timestamp > now) {
        return;
      }

      // Get the robot's current pose
      Pose2d robotPose = poseEstimator.getEstimatedPosition();

      // Reject large translation jumps
      double translationDistance = robotPose.getTranslation().getDistance(visionPose.getTranslation());
      if (translationDistance > DifferentialConstants.kVisionMaxTranslationJumpMeters) {
        return;
      }

      // Reject large rotation jumps
      double rotationDifference = Math.abs(robotPose.getRotation().minus(visionPose.getRotation()).getDegrees());
      if (rotationDifference > DifferentialConstants.kVisionMaxRotationJumpDegrees) {
        return;
      }

      // If we make it here => add vision measurement to pose estimator
      poseEstimator.addVisionMeasurement(
        visionPose,
        timestamp,
        VecBuilder.fill(stdDevs[0], stdDevs[1], stdDevs[2])
      );

      // Increment the count of accepted vision measurements for monitoring purposes
      addedVisionMeasurementCount++;
    } catch (Exception e) {
      Utils.logError("Error adding vision measurement: " + e.getMessage());
    }
  }

  // ==================== Internal State Modifiers ====================

  /**
   * Reset the robot's odometry
   */
  private void resetOdometry() {
    // Reset the gyro
    gyro.resetYaw();

    // Reset the encoders
    resetEncoders();

    // Reset the pose estimator to the origin
    poseEstimator.resetPosition(
      gyro.getRotation2d(),
      leftEncoder.getPosition(),
      rightEncoder.getPosition(),
      new Pose2d()
    );

    // Reset the count of added vision measurements
    addedVisionMeasurementCount = 0;
    
    Utils.logInfo("Odometry reset to origin");
  }

  /**
   * Drive the differential in arcade mode (used by driver in teleop)
   * @param xSpeed The forward/backward speed (-1.0 to 1.0)
   * @param rSpeed The rotation rate (-1.0 to 1.0)
   */
  private void driveArcade(double xSpeed, double rSpeed) {
    drive.arcadeDrive(
      MathUtil.clamp(xSpeed, -1.0, 1.0), 
      MathUtil.clamp(rSpeed, -1.0, 1.0)
    );
  }

  /**
   * Drive the differential in curvature mode (used by driver in teleop)
   * @param xSpeed The forward/backward speed (-1.0 to 1.0)
   * @param rSpeed The rotation rate (-1.0 to 1.0)
   * @param quickTurn Whether to enable quick turn mode
   */
  private void driveCurvature(double xSpeed, double rSpeed, boolean quickTurn) {
    drive.curvatureDrive(
      MathUtil.clamp(xSpeed, -1.0, 1.0),
      MathUtil.clamp(rSpeed, -1.0, 1.0),
      quickTurn
    );
  }

  /**
   * Drive the robot using robot-relative chassis speeds with feedforward and PID control.
   * @param speeds The desired robot-relative chassis speeds
   */
  private void driveRobotRelative(ChassisSpeeds speeds) {
    // Convert chassis speeds to wheel speeds
    DifferentialDriveWheelSpeeds targetSpeeds = kinematics.toWheelSpeeds(speeds);
    
    // Calculate feedforward
    double leftFF = feedForward.calculate(targetSpeeds.leftMetersPerSecond);
    double rightFF = feedForward.calculate(targetSpeeds.rightMetersPerSecond);
    
    // Calculate PID correction
    double leftPID = leftPIDController.calculate(leftEncoder.getVelocity(), targetSpeeds.leftMetersPerSecond);
    double rightPID = rightPIDController.calculate(rightEncoder.getVelocity(), targetSpeeds.rightMetersPerSecond);
    
    // Combine and set voltages
    double leftVoltage = MathUtil.clamp(leftFF + leftPID, -12.0, 12.0);
    double rightVoltage = MathUtil.clamp(rightFF + rightPID, -12.0, 12.0);
    
    // set the motor voltages directly for more precise control (feedforward + PID)
    leftLeaderMotor.setVoltage(leftVoltage);
    rightLeaderMotor.setVoltage(rightVoltage);

    // CRITICAL - Feed the motor safety watchdog
    drive.feed();
  }

  /**
   * Get the robot's current robot-relative chassis speeds
   * @return The current robot-relative chassis speeds
   */
  private ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getWheelSpeeds());
  }

  /**
   * Get the robot's current pose
   * @return The current estimated pose of the robot
   */
  private Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Reset the robot's pose to a specific location
   * @param pose The new pose to reset to
   */
  private void resetPose(Pose2d pose) {
    // Validate pose
    if (pose == null) {
      Utils.logError("Cannot reset to null pose");
      return;
    }
    
    // reset encoders
    resetEncoders();

    // reset pose estimator
    poseEstimator.resetPosition(
      gyro.getRotation2d(),
      leftEncoder.getPosition(),
      rightEncoder.getPosition(),
      pose
    );
    
    Utils.logInfo(String.format("Pose reset to: (%.2f, %.2f, %.2f°)", 
      pose.getX(), pose.getY(), pose.getRotation().getDegrees()));
  }

  /**
   * Get the current wheel speeds
   * @return The current wheel speeds
   */
  private DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      leftEncoder.getVelocity(),
      rightEncoder.getVelocity()
    );
  }

  /**
   * Reset the encoders to zero
   */
  private void resetEncoders() {
    leftEncoder.setPosition(0.0);
    rightEncoder.setPosition(0.0);
  }

  /**
   * Set the motor brake mode on all the motors
   * @param brake Whether to set brake (true) or coast (false) mode
   */
  private void setMotorBrake(boolean brake) {
    SparkMaxConfig config = new SparkMaxConfig();

    // set the idle mode
    config.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);

    // apply to all motors without resetting or persisting parameters
    rightLeaderMotor.configure(
      config, 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kNoPersistParameters
    );
    rightFollowerMotor.configure(
      config, 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kNoPersistParameters
    );
    leftLeaderMotor.configure(
      config, 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kNoPersistParameters
    );
    leftFollowerMotor.configure(
      config, 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kNoPersistParameters
    );
  }

  /**
   * Stop all the motors
   */
  private void stop() {
    drive.stopMotor();
  }

  // ==================== State Methods ====================
  
  /**
   * Get the average motor current
   * @return Average current in amps
   */
  private double getCurrent() {
    return (leftLeaderMotor.getOutputCurrent() + 
            leftFollowerMotor.getOutputCurrent() +
            rightLeaderMotor.getOutputCurrent() + 
            rightFollowerMotor.getOutputCurrent()) / 4.0;
  }
  
  /**
   * Get the average motor temperature
   * @return Average temperature in Celsius
   */
  private double getTemperature() {
    return (leftLeaderMotor.getMotorTemperature() + 
            leftFollowerMotor.getMotorTemperature() +
            rightLeaderMotor.getMotorTemperature() + 
            rightFollowerMotor.getMotorTemperature()) / 4.0;
  }
  
  /**
   * Get the average motor voltage
   * @return Average applied voltage
   */
  private double getVoltage() {
    return (leftLeaderMotor.getAppliedOutput() * leftLeaderMotor.getBusVoltage() +
            rightLeaderMotor.getAppliedOutput() * rightLeaderMotor.getBusVoltage()) / 2.0;
  }

  /**
   * Check if the drive controls are currently inverted
   * @return True if the drive controls are inverted, false otherwise
   */
  private boolean isInverted() {
    return inverted;
  }

  /**
   * Check if the drive is currently in slow mode
   * @return True if the drive is in slow mode, false otherwise
   */
  private boolean isSlowMode() {
    return slowMode;
  }

  /**
   * Initialize the drive for autonomous mode. This resets odometry, sets motor brake mode, 
   * and ensures controls are not inverted. This should be called at the start of 
   * autonomous to ensure the drive is in a known state.
   */
  public void autonomousInit() {
    resetOdometry();
    setMotorBrake(true);
    inverted = false;
  }

  /**
   * Initialize the drive for teleop mode. This resets motor brake mode, and ensures 
   * controls are not inverted. This should be called at the start of teleop to 
   * ensure the drive is in a known state.
   */
  public void teleopInit() {
    setMotorBrake(true);
    inverted = false;
  }

  /**
   * Initialize the drive for post-match mode. This resets motor brake mode, and ensures 
   * controls are not inverted. This should be called at the end of the match to 
   * ensure the drive is in a known state.
   */
  public void postMatchReset() {
    setMotorBrake(false);
    inverted = false;
  }

  /**
   * Get the distance to the current alliance hub
   * @return Distance in meters to the alliance hub
   */
  public double getDistanceToAllianceHub() {
    if (Utils.isRedAlliance()) {
      return getPose().getTranslation().getDistance(FieldConstants.kRedHubCenter);
    }
    return getPose().getTranslation().getDistance(FieldConstants.kBlueHubCenter);
  }

  // ==================== Command Factories ====================
  
  /**
   * Command to stop the drive
   * @return Command that stops the differential drive
   */
  public Command stopCommand() {
    return runOnce(this::stop)
      .withName("StopDifferential");
  }

  /**
   * Reset the robot's odometry
   * @return Command that resets the differential drive odometry
   */
  public Command resetOdometryCommand() {
    return runOnce(this::resetOdometry)
      .ignoringDisable(true)
      .withName("ResetOdometryDifferential");
  }

  /**
   * Command to set motor brake mode
   * @param brake Whether to enable brake mode
   * @return Command that sets the motor brake mode
   */
  public Command setMotorBrakeCommand(boolean brake) {
    return runOnce(() -> setMotorBrake(brake))
      .ignoringDisable(true)
      .withName("SetMotorBrakeDifferential");
  }
  
  /**
   * Command to toggle the drive controls inversion
   * @return Command that toggles the drive controls inversion
   */
  public Command toggleInvertControlsCommand() {
    return runOnce(() -> this.inverted = !this.inverted)
      .withName("ToggleInvertControlsDifferential");
  }
  
  /**
   * Command to toggle the drive controls slow mode
   * @return Command that toggles the drive controls slow mode
   */
  public Command toggleSlowModeCommand() {
    return runOnce(() -> this.slowMode = !this.slowMode)
      .withName("ToggleSlowModeDifferential");
  }
  
  /**
   * Command to set the drive controls slow mode on/off
   * @param slowMode Whether to enable slow mode true = on, false = off
   * @return Command that sets the drive controls slow mode
   */
  public Command setSlowModeCommand(boolean slowMode) {
    return runOnce(() -> this.slowMode = slowMode)
      .withName("SetSlowModeDifferential");
  }

  /**
   * Command to aim at the alliance hub using the gyro and pose estimation. 
   * This command will rotate the robot to face the hub and then end. It 
   * does not drive towards the hub, just aims at it. The command will 
   * timeout after 3 seconds to prevent getting stuck.
   * @return Command that aims the robot at the alliance hub using PID control
   */
  public Command aimAtHubCommand() {
    return run(() -> {
      // Get current pose
      Pose2d currentPose = getPose();

      // Get the current alliance hub
      Translation2d hubCenter = Utils.isRedAlliance() 
        ? FieldConstants.kRedHubCenter 
        : FieldConstants.kBlueHubCenter;
      
      // Calculate the angle from the robot to the hub
      Translation2d toHub = hubCenter.minus(currentPose.getTranslation());
      Rotation2d targetAngle = new Rotation2d(toHub.getX(), toHub.getY());
      
      // Calculate rotation speed to aim at the hub using the "aim" PID controller.
      // Add Math.PI so the rear-mounted launcher faces the hub.
      // The aim PID controller automatically handles:
      //  1. angle wrapping
      //  2. rotation acceleration constraints 
      //  3. and takes the shortest path to the target angle
      double rotationSpeed = aimPIDController.calculate(
        currentPose.getRotation().getRadians(),
        MathUtil.angleModulus(targetAngle.getRadians() + Math.PI)
      );

      // Clamp the rotation speed to the maximum rotational speed of the robot
      rotationSpeed = MathUtil.clamp(
        rotationSpeed, 
        -DifferentialConstants.kMaxAngularSpeedRadsPerSecond, 
        DifferentialConstants.kMaxAngularSpeedRadsPerSecond
      );

      // Drive the robot with the calculated rotation speed only
      driveRobotRelative(new ChassisSpeeds(0.0, 0.0, rotationSpeed));
    })
    .until(() -> aimPIDController.atSetpoint())
    .withTimeout(3.0)
    .finallyDo(this::stop)
    .withName("AimAtHubDifferential");
  }

  /**
   * Creates a command to drive to a specified pose using PathPlanner
   * @param targetPose The Pose2d to drive to
   * @return Command to drive via PathPlanner to the target pose
   */
  public Command driveToPoseCommand(Pose2d targetPose) {
    // Deferred command to get latest pose when scheduled
    // Set.of(this) ensures driveSubsystem is required
    return Commands.defer(() -> {
      // Check for null target pose
      if (targetPose == null) {
        Utils.logError("Cannot drive to null pose");
        return Commands.none();
      }

      // Create the constraints to use while pathfinding
      PathConstraints constraints = new PathConstraints(
        DifferentialConstants.kMaxSpeedMetersPerSecond, 
        DifferentialConstants.kMaxAccelMetersPerSecondSq,
        DifferentialConstants.kMaxAngularSpeedRadsPerSecond, 
        DifferentialConstants.kMaxAngularAccelRadsPerSecondSq
      );

      // Since AutoBuilder is configured, we can use it to build pathfinding commands
      return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
    }, Set.of(this))
    .withName("DriveToPoseDifferential");
  }

  /**
   * Command to drive the differential in arcade mode using power percentages (-1 to 1 range).
   * @param xSupplier The forward/backward speed supplier
   * @param rSupplier The rotation rate supplier
   * @return Command that drives the differential in arcade mode
   */
  public Command driveArcadeCommand(DoubleSupplier xSupplier, DoubleSupplier rSupplier) {
    return run(() -> {
      // 1. Apply deadband to the raw joystick inputs.
      //    This ignores noise from the joystick when it's in the neutral position.
      double xSpeed = MathUtil.applyDeadband(xSupplier.getAsDouble(), DifferentialConstants.kJoystickDeadband);
      double rSpeed = MathUtil.applyDeadband(rSupplier.getAsDouble(), DifferentialConstants.kJoystickDeadband);

      // 2. Apply slew rate limiters for a smoother acceleration ramp 
      xSpeed = xSpeedLimiter.calculate(xSpeed);
      rSpeed = rSpeedLimiter.calculate(rSpeed);

      // 3. Clamp the inputs to the maximum speeds of the robot
      if (isSlowMode()) {
        xSpeed = MathUtil.clamp(xSpeed, -DifferentialConstants.kMaxSlowModeSpeed, DifferentialConstants.kMaxSlowModeSpeed);
        rSpeed = MathUtil.clamp(rSpeed, -DifferentialConstants.kMaxSlowModeSpeed, DifferentialConstants.kMaxSlowModeSpeed);
      } else {
        xSpeed = MathUtil.clamp(xSpeed, -DifferentialConstants.kMaxTranslationalSpeed, DifferentialConstants.kMaxTranslationalSpeed);
        rSpeed = MathUtil.clamp(rSpeed, -DifferentialConstants.kMaxRotationalSpeed, DifferentialConstants.kMaxRotationalSpeed);
      }
    
      // 4. Invert controls if the inverted flag is set
      if (inverted) {
        xSpeed = -xSpeed;
        rSpeed = -rSpeed;
      }

      // 5. Drive the robot using the processed inputs (-1 to 1 range),
      //    arcadeDrive automatically squares inputs for finer control at low speeds
      driveArcade(xSpeed, rSpeed);
    }).withName("DriveArcadeDifferential");
  }

  /**
   * Command to drive the differential in curvature mode using power percentages (-1 to 1 range).
   * @param xSupplier The forward/backward speed supplier
   * @param rSupplier The rotation rate supplier
   * @return Command that drives the differential in curvature mode
   */
  public Command driveCurvatureCommand(DoubleSupplier xSupplier, DoubleSupplier rSupplier) {
    return run(() -> {
      // 1. Apply deadband to the raw joystick inputs.
      //    This ignores noise from the joystick when it's in the neutral position.
      double xSpeed = MathUtil.applyDeadband(xSupplier.getAsDouble(), DifferentialConstants.kJoystickDeadband);
      double rSpeed = MathUtil.applyDeadband(rSupplier.getAsDouble(), DifferentialConstants.kJoystickDeadband);

      // Determine if we should be in quick turn mode (allows for sharper turns at low speeds)
      boolean quickTurn = Math.abs(xSupplier.getAsDouble()) < DifferentialConstants.kQuickTurnThreshold;

      // 2. Square inputs for finer low-speed control while preserving sign
      xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
      rSpeed = Math.copySign(rSpeed * rSpeed, rSpeed);

      // 3. Apply slew rate limiters for a smoother acceleration ramp 
      xSpeed = xSpeedLimiter.calculate(xSpeed);
      rSpeed = rSpeedLimiter.calculate(rSpeed);

      // 4. Clamp the inputs to the maximum speeds of the robot
      if (isSlowMode()) {
        xSpeed = MathUtil.clamp(xSpeed, -DifferentialConstants.kMaxSlowModeSpeed, DifferentialConstants.kMaxSlowModeSpeed);
        rSpeed = MathUtil.clamp(rSpeed, -DifferentialConstants.kMaxSlowModeSpeed, DifferentialConstants.kMaxSlowModeSpeed);
      } else {
        xSpeed = MathUtil.clamp(xSpeed, -DifferentialConstants.kMaxTranslationalSpeed, DifferentialConstants.kMaxTranslationalSpeed);
        rSpeed = MathUtil.clamp(rSpeed, -DifferentialConstants.kMaxRotationalSpeed, DifferentialConstants.kMaxRotationalSpeed);
      }
    
      // 5. Invert controls if the inverted flag is set
      if (inverted) {
        xSpeed = -xSpeed;
        rSpeed = -rSpeed;
      }

      // 6. Drive the robot using the processed inputs (-1 to 1 range),
      driveCurvature(xSpeed, rSpeed, quickTurn);
    }).withName("DriveCurvatureDifferential");
  }

  // ==================== Telemetry Methods ====================

  /**
   * Updates the SmartDashboard with the robot's readiness for autonomous mode.
   * This should be called perioddically prior to autonomous mode while the robot is disabled.
   * Team members then move the robot left/right/back/forward/rotate until the all the errors are zero.
   * @param expectedStart The expected starting pose for the selected autonomous mode
   */
  public void displayAutoReadiness(Pose2d expectedStart) {
    if (expectedStart == null) {
      return;
    }

    // get the robot's current pose
    Pose2d currentPose = getPose();

    // calculate the errors between the current pose and the expected starting pose
    double xError = expectedStart.getX() - currentPose.getX();
    double yError = expectedStart.getY() - currentPose.getY();
    double angleError = expectedStart.getRotation().minus(currentPose.getRotation()).getDegrees();

    boolean inPosition = Math.abs(xError) < 0.1 && 
                         Math.abs(yError) < 0.1 &&
                         Math.abs(angleError) < 2.0;

    // display if the robot is in the correct starting position for autonomous
    SmartDashboard.putBoolean("Auto/RobotInPosition", inPosition);

    // display the robot's current pose according to the pose estimator
    SmartDashboard.putNumber("Auto/Current X (m)", Utils.showDouble(currentPose.getX()));
    SmartDashboard.putNumber("Auto/Current Y (m)", Utils.showDouble(currentPose.getY()));
    SmartDashboard.putNumber("Auto/Current Angle (deg)", Utils.showDouble(currentPose.getRotation().getDegrees()));

    // display the expected starting pose for the selected autonomous mode (from the auto chooser)
    SmartDashboard.putNumber("Auto/Target X (m)", Utils.showDouble(expectedStart.getX()));
    SmartDashboard.putNumber("Auto/Target Y (m)", Utils.showDouble(expectedStart.getY()));
    SmartDashboard.putNumber("Auto/Target Angle (deg)", Utils.showDouble(expectedStart.getRotation().getDegrees()));

    // display the errors between the current pose and the expected starting pose
    SmartDashboard.putNumber("Auto/X Error (m)", Utils.showDouble(xError));
    SmartDashboard.putNumber("Auto/Y Error (m)", Utils.showDouble(yError));
    SmartDashboard.putNumber("Auto/Angle Error (deg)", Utils.showDouble(angleError));
  }

  /**
   * Initialize Sendable for SmartDashboard
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("DifferentialSubsystem");
    builder.addDoubleProperty("Left Position (m)", () -> Utils.showDouble(leftEncoder.getPosition()), null);
    builder.addDoubleProperty("Right Position (m)", () -> Utils.showDouble(rightEncoder.getPosition()), null);
    builder.addDoubleProperty("Left Velocity (mps)", () -> Utils.showDouble(leftEncoder.getVelocity()), null);
    builder.addDoubleProperty("Right Velocity (mps)", () -> Utils.showDouble(rightEncoder.getVelocity()), null);
    builder.addDoubleProperty("Gyro Angle (deg)", () -> Utils.showDouble(gyro.getAngle().in(Degrees)), null);
    builder.addDoubleProperty("Dist To Hub (m)", () -> Utils.showDouble(getDistanceToAllianceHub()), null);
    builder.addDoubleProperty("Voltage (V)", () -> Utils.showDouble(getVoltage()), null);
    builder.addDoubleProperty("Current (A)", () -> Utils.showDouble(getCurrent()), null);
    builder.addDoubleProperty("Temperature (C)", () -> Utils.showDouble(getTemperature()), null);
    builder.addIntegerProperty("Added Vision Measurements", () -> addedVisionMeasurementCount, null);
    builder.addBooleanProperty("Inverted", this::isInverted, null);
    builder.addBooleanProperty("Slow Mode", this::isSlowMode, null);
  }
}
