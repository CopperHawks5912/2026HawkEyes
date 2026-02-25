// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.fuel;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class FuelConstants {
  // Feeder motor percentages (for brushed motor)
  public static final double kFeederIntakingPercent         = -0.80; 
  public static final double kFeederLaunchingPercent        =  0.60;
  public static final double kFeederPassingPercent          =  0.60;
  public static final double kFeederSpinUpPreLaunchPercent  = -0.50;

  // Intake/Launcher percentages (for intake/eject mode - percentage control)
  public static final double kIntakeIntakingPercent         =  0.60;
  public static final double kIntakeEjectPercent            = -0.80;

  // Launcher RPM values (for shooting mode - velocity control)
  public static final double kLauncherLaunchingRPM          =  4500.0;  // Default launch RPM
  public static final double kLauncherPassingRPM            =  2000.0;  // Passing RPM
  
  // Launcher velocity control PID constants
  public static final double kLauncherP                     =  0.0002;
  public static final double kLauncherI                     =  0.0;
  public static final double kLauncherD                     =  0.0;

  // Feedforward constants (Modern REV API)
  // These replace the deprecated velocityFF() method
  // Use SysId or empirical tuning to find these values
  public static final double kLauncherKS                    =  0.0;      // Static friction (volts) - voltage to overcome friction
  public static final double kLauncherKV                    =  0.00167;  // Velocity constant (volts per RPM) - was 1/6000
  public static final double kLauncherKA                    =  0.0;      // Acceleration constant (volts per RPM/s) - usually small for flywheels
  
   // Note: kV = 1 / free_speed_rpm when battery is at 12V
  // Example: If free speed is 6000 RPM at 12V, then kV = 12/6000 = 0.002 V/(RPM)
  // Or in the units the REV API expects: kV = 1/6000 = 0.00167 (assuming 12V compensation)  

  public static final double kLauncherToleranceRPM          =  150.0;

  // Timing constants
  public static final double kSpinUpSeconds                 =  0.75;
  public static final double kRecoveryDelaySeconds          =  0.15;
}
