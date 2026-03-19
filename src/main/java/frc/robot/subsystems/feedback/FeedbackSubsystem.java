// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feedback;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.PWMConstants;
import frc.robot.subsystems.feedback.FeedbackConstants.DisplayMode;
import frc.robot.util.Utils;

/**
 * Since only one command can "require" the subsystem at a time, we need to separate 
 * LED feedback from rumble feedback for them to co-exist in this feedback subsystem.
 * Otherwise, a rumble command would interrupt any LED display command and vice 
 * versa, which would lead to a poor user experience.
 * 
 * Therefore:
 *  a. LED display is handled by the periodic loop that changes the LED buffer 
 *     based on the current display mode (currentMode) and animation state. 
 *     This is why the setDisplayCommand uses "runOnce" to just set the display mode, 
 *     and the periodic loop takes care of updating the LED buffer accordingly.
 *  
 *  b. Rumble feedback is handled by "run" commands that "require" this subsystem
 *     for the duration of their execution.
 * 
 *  NOTE: When combining LED and rumble commands, make sure to first set the display mode
 *        as this finishes instantly. Then use "andThen" to chain on rumble commands 
 *        so they can run sequentially without interrupting each other.
 */
public class FeedbackSubsystem extends SubsystemBase {
  // LED hardware
  private final AddressableLED ledStrip;
  private final AddressableLEDBuffer ledBuffer;
  
  // Controller reference for rumble
  private CommandXboxController controller;
  
  // Display state
  private DisplayMode currentMode = DisplayMode.OFF;
  private DisplayMode previousMode = DisplayMode.OFF; // For reverting after temporary displays
  private double animationTimer = 0;
  private int animationOffset = 0;
  private char gameData = '?';
      
  /**
   * Creates a new FeedbackSubsystem.
   * @param controller The Xbox controller to use for rumble feedback
   */
  public FeedbackSubsystem(CommandXboxController controller) {
    // Set controller reference
    this.controller = controller;

    // Initialize LED strip
    ledBuffer = new AddressableLEDBuffer(FeedbackConstants.LEDLength);    
    ledStrip = new AddressableLED(PWMConstants.kLEDStringID);
    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();
    
    // Output initialization progress
    Utils.logInfo("Feedback subsystem initialized");
  }

  @Override
  public void periodic() {
    updateLEDDisplay();
    ledStrip.setData(ledBuffer);
  }

  // ==================== LED Control Methods ====================

  /**
   * Set the current LED display mode
   * @param mode Display mode to show
   */
  private void setDisplayMode(DisplayMode mode) {
    currentMode = mode;
    animationTimer = 0;
    animationOffset = 0;
  }
  
  /**
   * Update the LED buffer based on current display mode
   */
  private void updateLEDDisplay() {
    animationTimer += 0.02; // Assuming 20ms periodic cycle
    
    switch (currentMode) {
      case OFF:
        setAllLEDs(Color.kBlack);
        break;

      case AIMED_AT_HUB:
        blinkPattern(FeedbackConstants.AimedAtHubColor, 0.50);
        break;
        
      case IDLE:
        chasePattern(FeedbackConstants.IdleColor, 0.50);
        break;
        
      case INFO:
        blinkPattern(FeedbackConstants.InfoColor, 0.50);
        break;
        
      case WARNING:
        blinkPattern(FeedbackConstants.WarningColor, 0.25);
        break;
        
      case ERROR:
        blinkPattern(FeedbackConstants.ErrorColor, 0.15);
        break;
        
      case TEAM_COLORS:
        teamColorsPattern();
        break;

      case CANDY_CANE:
        candyCanePattern();
        break;

      case FUNKY_DISCO:
        funkyDiscoPattern();
        break;

      case SCORING_SHIFT:
        scoringShiftPattern();
        break;
    }
  }
  
  /**
   * Set all LEDs to a single color
   * @param color Color to set
   */
  private void setAllLEDs(Color color) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setLED(i, color);
    }
  }
  
  /**
   * Pulse pattern that fades in and out
   * @param color Base color
   * @param period Time for one complete pulse cycle in seconds
   */
  private void pulsePattern(Color color, double period, double timeOffset) {
    double brightness = (Math.sin((animationTimer - timeOffset) * 2 * Math.PI / period) + 1) / 2;
    Color scaledColor = new Color(
      color.red * brightness,
      color.green * brightness,
      color.blue * brightness
    );
    setAllLEDs(scaledColor);
  }
  
  /**
   * Blink pattern that turns on and off
   * @param color Color to blink
   * @param period Time for one complete blink cycle in seconds
   */
  private void blinkPattern(Color color, double period) {
    boolean on = (animationTimer % period) < (period / 2);
    setAllLEDs(on ? color : Color.kBlack);
  }
  
  /**
   * Chase pattern that moves along the strip
   * @param color Color to chase
   * @param speed Speed of the chase (lower = faster)
   */
  private void chasePattern(Color color, double speed) {
    // Calculate offset directly from time
    animationOffset = (int)(animationTimer / speed) % ledBuffer.getLength();
    
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      if ((i + animationOffset) % 5 == 0) {
        ledBuffer.setLED(i, color);
      } else {
        ledBuffer.setLED(i, Color.kBlack);
      }
    }
  }
  
  /**
   * Team colors pattern that fades orange in/out then green in/out repeatedly
   */
  private void teamColorsPattern() {
    double phaseDuration = 1.5; // seconds per full pulse
    double cycleTime = animationTimer % (phaseDuration * 2);

    if (cycleTime < phaseDuration) {
      pulsePattern(FeedbackConstants.TeamCopper, phaseDuration, 0);
    } else {
      pulsePattern(FeedbackConstants.TeamGreen, phaseDuration, phaseDuration);
    }
  }
  
  /**
   * Candy cane pattern that alternates red and white stripes and rotates
   */
  private void candyCanePattern() {
    // Calculate offset for rotation effect
    animationOffset = (int)(animationTimer * 10) % ledBuffer.getLength(); // 10 pixels/second
    
    // Stripe width (number of LEDs per stripe)
    int stripeWidth = 3;
    
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      // Calculate position with animation offset for rotation effect
      int position = (i + animationOffset) % ledBuffer.getLength();
      
      // Determine which stripe we're in
      boolean isRedStripe = ((position / stripeWidth) % 2) == 0;
      
      // Set color (red or white)
      ledBuffer.setLED(i, isRedStripe ? Color.kRed : Color.kWhite);
    }
  }

  /**
   * Funky disco pattern with random flashing colors
   */
  private void funkyDiscoPattern() {
    // Fast beat - change every 0.1 seconds
    int beatPhase = (int)(animationTimer / 0.1);
    
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      // Create pseudo-random but repeatable colors based on position and time
      // This ensures the same position gets the same color during the same beat
      int seed = (i * 7 + beatPhase * 13) % 360;
      
      // Convert to HSV for vibrant colors
      // Hue: based on seed (full spectrum)
      // Saturation: 100% (fully saturated = vibrant)
      // Value: randomly bright or dark for flashing effect
      int hue = seed;
      double saturation = 1.0;
      double value = ((seed * 3) % 2 == 0) ? 1.0 : 0.3; // Alternate bright/dim
      
      // Convert HSV to RGB
      Color color = Color.fromHSV(hue, (int)(saturation * 255), (int)(value * 255));
      ledBuffer.setLED(i, color);
    }
  }

  /**
   * Scoring shift pattern that indicates which alliance is allowed to score.
   * Sets all LEDs to our alliance colour when our scoring shift is activate.
   * Blinks the appropriate colour for the last 5 seconds of each hub shift.
   * Sets LEDs to yellow (collect fuel) when it is not our turn to score.
   */
  private void scoringShiftPattern() {
    // Use real match time if FMS attached, otherwise simulate from FPGA timestamp
    double time = DriverStation.isFMSAttached() 
      ? DriverStation.getMatchTime()
      : 140.0 - (Timer.getFPGATimestamp() % 140.0); // counts down from 140 seconds

    // set our alliance color
    Color allianceColor = Utils.isRedAlliance() ? Color.kRed : Color.kBlue;

    // color to indicate inactive hub (collect fuel)
    Color inactiveColor = Color.kYellow;

    // if no game data, just show alliance color solid
    if (gameData == '?') {
      setAllLEDs(allianceColor);
      return;
    }

    // autonomous period 20 seconds (both alliances can score)
    if (gameData == 'A') {
      if (time <= 20 && time > 5) {
        // autonomous (both alliances can score)
        setAllLEDs(allianceColor);
      }
      else if (time <= 5 && time > 0) {
        // autonomous - last 5 seconds (both alliances can score)
        pulsePattern(allianceColor, 0.35, 0);
      }
      else {
        // default to off
        setAllLEDs(allianceColor);
      }
    }

    // check that we have valid game data
    if (gameData == 'R' || gameData == 'B') {
      // check if our alliance is inactive for the 1st shift
      boolean isInactiveFirst = 
        (gameData == 'R' && Utils.isRedAlliance()) ||
        (gameData == 'B' && !Utils.isRedAlliance());

      // shift data from 2026 FRC Game Manual
      // https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf
      if (time <= 140 && time > 135) {
        // transition shift (both alliances can score)
        setAllLEDs(allianceColor);
      }
      else if (time <= 135 && time > 130) {
        // transition shift - last 5 seconds (both alliances can score)
        pulsePattern(allianceColor, 0.35, 0);
      }
      else if (time <= 130 && time > 110) {
        // shift 1
        setAllLEDs(isInactiveFirst ? inactiveColor : allianceColor);
      }
      else if (time <= 110 && time > 105) {
        // shift 1 - last 5 seconds
        pulsePattern(isInactiveFirst ? inactiveColor : allianceColor, 0.35, 0);
      }
      else if (time <= 105 && time > 85) {
        // shift 2
        setAllLEDs(isInactiveFirst ? allianceColor : inactiveColor);
      }
      else if (time <= 85 && time > 80) {
        // shift 2 - last 5 seconds
        pulsePattern(isInactiveFirst ? allianceColor : inactiveColor, 0.35, 0);
      }
      else if (time <= 80 && time > 60) {
        // shift 3
        setAllLEDs(isInactiveFirst ? inactiveColor : allianceColor);
      }
      else if (time <= 60 && time > 55) {
        // shift 3 - last 5 seconds
        pulsePattern(isInactiveFirst ? inactiveColor : allianceColor, 0.35, 0);
      }
      else if (time <= 55 && time > 35) {
        // shift 4
        setAllLEDs(isInactiveFirst ? allianceColor : inactiveColor);
      }
      else if (time <= 35 && time > 30) {
        // shift 4 - last 5 seconds
        pulsePattern(isInactiveFirst ? allianceColor : inactiveColor, 0.35, 0);
      }
      else if (time <= 30 && time > 5) {
        // end game (both alliances can score)
        setAllLEDs(allianceColor);
      }
      else if (time <= 5 && time > 0) {
        // end game - last 5 seconds (both alliances can score)
        pulsePattern(allianceColor, 0.35, 0);
      }
      else {
        // default to off
        setAllLEDs(Color.kBlack);
      }
    }
  }

  // ==================== Rumble Control Methods ====================
  
  /**
   * Set controller rumble
   * @param intensity Rumble intensity (0.0 to 1.0)
   */
  private void setRumble(double intensity) {
    if (controller != null) {
      double clampedIntensity = MathUtil.clamp(intensity, 0.0, 1.0);
      controller.getHID().setRumble(RumbleType.kBothRumble, clampedIntensity);
    }
  }
  
  /**
   * Stop controller rumble
   */
  private void stopRumble() {
    setRumble(0);
  }
  
  // ==================== Command Factories ====================
  
  /**
   * Command to set LED display mode
   * @param mode Display mode to show
   * @return Command that sets the LED display
   */
  public Command setDisplayCommand(DisplayMode mode) {
    return runOnce(() -> setDisplayMode(mode))
      .ignoringDisable(true)
      .withName("SetLED_" + mode.name() + "_Feedback");
  }
  
  /**
   * Command to rumble controller for a duration
   * @param intensity Rumble intensity (0.0 to 1.0)
   * @param duration Duration in seconds
   * @return Command that rumbles then stops
   */
  public Command rumbleCommand(double intensity, double duration) {
    return run(() -> setRumble(intensity))
      .withTimeout(duration)
      .finallyDo(this::stopRumble)
      .withName("RumbleFeedback");
  }
  
  /**
   * Command for short rumble pulse
   * @return Command that does a quick rumble
   */
  public Command quickRumbleCommand() {
    return rumbleCommand(0.5, 0.2)
      .withName("QuickRumbleFeedback");
  }
  
  /**
   * Command for strong rumble pulse
   * @return Command that does a strong rumble
   */
  public Command strongRumbleCommand() {
    return rumbleCommand(1.0, 0.4)
      .withName("StrongRumbleFeedback");
  }
  
  /**
   * Command for double rumble pulse
   * @return Command that rumbles twice
   */
  public Command doubleRumbleCommand() {
    return rumbleCommand(0.7, 0.15)
      .andThen(Commands.waitSeconds(0.1))
      .andThen(rumbleCommand(0.7, 0.15))
      .withName("DoubleRumbleFeedback");
  }
  
  /**
   * Command to set off feedback
   * @return Command that sets LED display off
   */
  public Command offCommand() {
    return setDisplayCommand(DisplayMode.OFF)
      .withName("OffFeedback");
  }
  
  /**
   * Command to indicate error state for a short duration and
   * then revert to the previous display mode.
   * @return Command with error LED and rumble feedback
   */
  public Command errorCommand() {
    return runOnce(() -> {
      if (currentMode != DisplayMode.ERROR) {
        previousMode = currentMode;
      }
      setDisplayMode(DisplayMode.ERROR);
    })
    .andThen(strongRumbleCommand())
    .andThen(Commands.waitSeconds(0.6))
    .andThen(setDisplayCommand(previousMode))
    .withName("ErrorFeedback");
  }
  
  /**
   * Command to indicate warning state for a short duration and
   * then revert to the previous display mode.
   * @return Command with warning LED and rumble feedback
   */
  public Command warningCommand() {
    return runOnce(() -> {
      if (currentMode != DisplayMode.WARNING) {
        previousMode = currentMode;
      }
      setDisplayMode(DisplayMode.WARNING);
    })
    .andThen(rumbleCommand(0.4, 0.3))
    .andThen(Commands.waitSeconds(0.7))
    .andThen(setDisplayCommand(previousMode))
    .withName("WarningFeedback");
  }
  
  /**
   * Command to indicate info state for a short duration and
   * then revert to the previous display mode.
   * @return Command with info LED and rumble feedback
   */
  public Command infoCommand() {
    return runOnce(() -> {
      if (currentMode != DisplayMode.INFO) {
        previousMode = currentMode;
      }
      setDisplayMode(DisplayMode.INFO);
    })
    .andThen(quickRumbleCommand())
    .andThen(Commands.waitSeconds(0.8))
    .andThen(setDisplayCommand(previousMode))
    .withName("InfoFeedback");
  }
  
  /**
   * Command to set idle feedback
   * @return Command that sets idle LED display
   */
  public Command idleCommand() {
    return setDisplayCommand(DisplayMode.IDLE)
      .withName("IdleFeedback");
  }
  
  /**
   * Command to display team colors gradient
   * @return Command that pulses between green & copper
   */
  public Command teamColorsCommand() {
    return setDisplayCommand(DisplayMode.TEAM_COLORS)
      .withName("TeamColorsFeedback");
  }

  /**
   * Command to display candy cane pattern
   * @return Command that shows rotating red and white stripes
   */
  public Command candyCaneCommand() {
    return setDisplayCommand(DisplayMode.CANDY_CANE)
      .withName("CandyCaneFeedback");
  }

  /**
   * Command to display funky disco pattern
   * @return Command that shows random flashing rainbow colors
   */
  public Command funkyDiscoCommand() {
    return setDisplayCommand(DisplayMode.FUNKY_DISCO)
      .withName("FunkyDiscoFeedback");
  }

  /**
   * Command to display scoring shift feedback
   * @return Command that sets the scoring shift display mode
   */
  public Command scoringShiftCommand() {
    return setDisplayCommand(DisplayMode.SCORING_SHIFT)
      .withName("ScoringShiftFeedback");
  }
  
  /**
   * Command to display aimed at hub feedback
   * @return Command that shows aimed at hub LED display and rumble
   */
  public Command aimedAtHubCommand() {
    return runOnce(() -> {
      if (currentMode != DisplayMode.AIMED_AT_HUB) {
        previousMode = currentMode;
      }
      setDisplayMode(DisplayMode.AIMED_AT_HUB);
    })
    .andThen(doubleRumbleCommand())
    .andThen(Commands.waitSeconds(0.6))
    .andThen(setDisplayCommand(previousMode))
    .withName("AimedAtHubFeedback");
  }
  
  /**
   * Command to set game data and then start scoring shift feedback
   * @param gameData Character indicating which alliance will be inactive at the start of teleop
   * @return Command that sets the scoring shift display mode
   */
  public Command setGameDataCommand(char gameData) {
    return runOnce(() -> { this.gameData = gameData; })
      .andThen(scoringShiftCommand())
      .withName("SetGameDataFeedback");
  }
}
