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
  // Feeder motor percentages
  public static final double kFeederLaunchingPercent        =  0.60; // Feed fuel into the launcher for launcher
  public static final double kFeederPassingPercent          =  0.60; // Feed fuel into the launcher for passing
  public static final double kFeederIntakingPercent         = -0.60; // A reverse power to pull fuel into the hopper
  public static final double kFeederEjectingPercent         =  0.60; // Feed fuel towards the intake to eject fuel

  // Launcher & intake percentages (physically same motors)
  public static final double kLauncherLaunchingPercent      =  0.80; // Power for launching
  public static final double kLauncherPassingPercent        =  0.60; // Power for passing
  public static final double kLauncherIntakingPercent       =  0.60; // Power for intaking
  public static final double kLauncherEjectingPercent       = -0.80; // Power for ejecting
  
  // hold back fuel during launcher spinup
  public static final double kFeederSpinUpPreLaunchPercent  =  0.0; // was -0.25
  
  // Wait a short time for the launcher to spin up before feeding fuel
  public static final double kLauncherSpinUpTimeoutSeconds  =  0.75;

  // =========================================================================
  // NOTE: Not using velocity control, left here for reference
  // =========================================================================

  // Launcher RPM velocity values (for shooting mode - velocity PID control)
  public static final double kLauncherPassingRPM            = 3000.0; // Passing RPM
  
  // Launcher velocity control PID constants
  public static final double kLauncherP                     = 0.0002;
  public static final double kLauncherI                     = 0.0;
  public static final double kLauncherD                     = 0.0;

  // Feedforward constants (Modern REV API)
  // These replace the deprecated velocityFF() method
  // Use SysId or empirical tuning to find these values
  public static final double kLauncherKS                    = 0.1;      // Static friction (volts) - voltage to overcome friction
  public static final double kLauncherKV                    = 0.00211;  // 12/5676 with voltage compensation
  public static final double kLauncherKA                    = 0.0;      // Acceleration constant (volts per RPM/s) - usually small for flywheels
  
  // Note: kV = 1 / free_speed_rpm when battery is at 12V
  // Example: If free speed is 6000 RPM at 12V, then kV = 12/6000 = 0.002 V/(RPM)
  // Or in the units the REV API expects: kV = 1/6000 = 0.00167 (assuming 12V compensation)  

  public static final double kLauncherToleranceRPM          = 100.0;
}
