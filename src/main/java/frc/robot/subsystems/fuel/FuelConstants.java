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
  // Wait a short time for the launcher to spin up before feeding fuel.
  // Used by both simple motor percentage control and RPM velocity control versions of the code.
  public static final double kLauncherSpinUpTimeoutSeconds  =  0.50;

  // -------------------------------------------------------------------------
  // Tune these values when using simple motor percentage control
  // -------------------------------------------------------------------------
  // Feeder motor percentages
  public static final double kFeederLaunchingPercent        =  0.60;
  public static final double kFeederIntakingPercent         = -0.80; // must be negative
  public static final double kFeederEjectingPercent         =  0.60;

  // Launcher & intake percentages (physically same motors)
  public static final double kLauncherLaunchingPercent      =  0.80;
  public static final double kLauncherIntakingPercent       =  0.50;
  public static final double kLauncherEjectingPercent       = -0.80; // must be negative
  
  // hold back fuel during launcher spinup
  public static final double kFeederSpinUpPreLaunchPercent  = -0.25; // must be negative

  // -------------------------------------------------------------------------
  // Tune these values when using RPM velocity control
  // -------------------------------------------------------------------------
  // Feeder motor RPMs [-6380, 6380] for Falcon 500 - convert to RPS for motor controller
  public static final double kFeederLaunchingRPS            =  3828 / 60.0;
  public static final double kFeederIntakingRPS             = -5104 / 60.0; // must be negative
  public static final double kFeederEjectingRPS             =  3828 / 60.0;
  
  // Launcher & intake RPMs [-6000, 6000] for Kraken x60 - convert to RPS for motor controller
  public static final double kLauncherLaunchingRPS          =  4800 / 60.0;
  public static final double kLauncherIntakingRPS           =  3000 / 60.0;
  public static final double kLauncherEjectingRPS           = -4800 / 60.0; // must be negative
  
  // hold back fuel during launcher spinup
  public static final double kFeederSpinUpPreLaunchRPS      =  -240 / 60.0; // must be negative

  // Launcher velocity control PID constants
  public static final double kLauncherP = 0.012; 
  public static final double kLauncherI = 0.0;
  public static final double kLauncherD = 0.0;
  public static final double kLauncherS = 0.1;    // Static friction (volts) - voltage to overcome friction
  public static final double kLauncherV = 0.12;   // 12/100 with voltage compensation
  public static final double kLauncherA = 0.0;    // Acceleration constant (volts per RPS/s)
  
  // Feeder velocity control PID constants
  public static final double kFeederP   = 0.012;
  public static final double kFeederI   = 0.0;
  public static final double kFeederD   = 0.0;
  public static final double kFeederS   = 0.1;    // Static friction (volts) - voltage to overcome friction
  public static final double kFeederV   = 0.113;  // 12/106 with voltage compensation
  public static final double kFeederA   = 0.0;    // Acceleration constant (volts per RPS/s)
}
