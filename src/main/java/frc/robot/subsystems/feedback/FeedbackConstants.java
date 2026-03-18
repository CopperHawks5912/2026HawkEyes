// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feedback;

import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class FeedbackConstants {
  /** Number of LEDs in the strip */
  public static final int LEDLength = 60;

  /** Color for indicating that the robot is aimed at the hub */
  public static final Color AimedAtHubColor = Color.kGreen;
      
  /** Color when robot is idle/ready (soft blue) */
  public static final Color IdleColor = new Color(0.0, 0.3, 1.0);
      
  /** Color for informational messages (cyan) */
  public static final Color InfoColor = new Color(0.0, 0.3, 1.0);

  /** Color for warnings (orange) */
  public static final Color WarningColor = new Color(1.0, 0.5, 0.0);

  /** Color for errors (red) */
  public static final Color ErrorColor = Color.kRed;
      
  /** Team color - Green */
  public static final Color TeamGreen = new Color(0.0, 1.0, 0.0);

  /** Team color - Copper */
  public static final Color TeamCopper = new Color(0.72, 0.45, 0.20);
      
  /**
   * Enum representing different LED display modes
   */
  public enum DisplayMode {
    /** All LEDs off */
    OFF,
    
    /** Robot is aimed at the hub */
    AIMED_AT_HUB,
    
    /** Robot is idle/ready state */
    IDLE,

    /** Informational state */
    INFO,
    
    /** Warning state */
    WARNING,
    
    /** Error state */
    ERROR,
    
    /** Team colors gradient chase */
    TEAM_COLORS,

    /** Candy cane pattern */
    CANDY_CANE,

    /** Funky disco mode */
    FUNKY_DISCO,

    /** Scoring shift indication */
    SCORING_SHIFT
  }
}
