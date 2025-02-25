// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Intake.IntakeItem;
import frc.robot.subsystems.Intake.IntakeState;

public class Lights extends SubsystemBase {
  /** Creates a new Lights. */
  private String fieldSide = "none";
  private boolean commandRunning = false;
  private double time = Timer.getFPGATimestamp();
  private double timeout = 0.0;
  private boolean timedFlashes = false;
  private double strobeSpeed = 0.4;
  private double flashSpeed = 0.05;
  CANdle candleSwerve = new CANdle(Constants.CANInfo.CANDLE_ID_0, "Canivore");
  CANdle candleBack = new CANdle(Constants.CANInfo.CANDLE_ID_1, "rio");
  CANdle candleFront = new CANdle(Constants.CANInfo.CANDLE_ID_2, "rio");
  private int ledNumber = 2000;
  private int ledsPerSwerve = 3000;

  // ColorFlowAnimation redSolidDim = new ColorFlowAnimation(100, 0, 0, 0, 1.0,
  // ledNumber, Direction.Forward, 0);
  // ColorFlowAnimation redSolidBright = new ColorFlowAnimation(255, 0, 0, 0, 0.0,
  // ledNumber, Direction.Forward, 0);
  // ColorFlowAnimation blueSolidDim = new ColorFlowAnimation(0, 0, 100, 0, 0.0,
  // ledNumber, Direction.Forward, 0);
  // ColorFlowAnimation blueSolidBright = new ColorFlowAnimation(0, 0, 255, 0,
  // 0.0, ledNumber, Direction.Forward, 0);

  // FireAnimation redSolidDim = new FireAnimation(0.5, 1.0, ledNumber, 0.0, 0.0,
  // false, 0);
  // LarsonAnimation redSolidDim = new LarsonAnimation(100, 0, 0, 0, 0.5,
  // ledNumber, BounceMode.Back, 1000, 0);

  // ColorFlowAnimation greenSolidBright = new ColorFlowAnimation(0, 255, 0, 0,
  // 0.0, ledNumber, Direction.Forward, 0);
  // ColorFlowAnimation whiteSolidBright = new ColorFlowAnimation(255, 255, 255,
  // 255, 0.0, ledNumber, Direction.Forward,
  // 0);

  StrobeAnimation redFlash = new StrobeAnimation(255, 0, 0, 0, 0.1, ledNumber, 0);
  StrobeAnimation blueFlash = new StrobeAnimation(0, 0, 255, 0, 0.1, ledNumber, 0);

  StrobeAnimation greenStrobe = new StrobeAnimation(0, 255, 0, 0, strobeSpeed, ledNumber, 0);
  StrobeAnimation purpleStrobe = new StrobeAnimation(100, 0, 100, 0, strobeSpeed, ledNumber, 0);
  StrobeAnimation yellowStrobe = new StrobeAnimation(100, 100, 0, 0, strobeSpeed, ledNumber, 0);

  StrobeAnimation greenFlash = new StrobeAnimation(0, 255, 0, 0, flashSpeed, ledNumber, 0);
  StrobeAnimation purpleFlash = new StrobeAnimation(100, 0, 100, 0, flashSpeed, ledNumber, 0);
  StrobeAnimation yellowFlash = new StrobeAnimation(100, 100, 0, 0, flashSpeed, ledNumber, 0);

  // ColorFlowAnimation algaeSolid = new ColorFlowAnimation(0, 255, 150, 0, 0.0,
  // ledNumber, Direction.Forward, 0);
  // ColorFlowAnimation coralSolid = new ColorFlowAnimation(255, 255, 255, 255,
  // 0.0, ledNumber, Direction.Forward, 0);
  StrobeAnimation algaeFlashing = new StrobeAnimation(0, 75, 25, 0, flashSpeed, ledNumber, 0);
  StrobeAnimation coralFlashing = new StrobeAnimation(50, 50, 50, 50, flashSpeed, ledNumber, 0);
  StrobeAnimation algaeStrobing = new StrobeAnimation(0, 75, 25, 0, strobeSpeed, ledNumber, 0);
  StrobeAnimation coralStrobing = new StrobeAnimation(50, 50, 50, 50, strobeSpeed, ledNumber, 0);

  LarsonAnimation coralKnightRiderAnimation = new LarsonAnimation(255, 255, 255, 0, 0.8, ledNumber,
      BounceMode.Back,
      100, 0);
  LarsonAnimation algaeKnightRiderAnimation = new LarsonAnimation(0, 255, 100, 0, 0.8, ledNumber,
      BounceMode.Back,
      100, 0);

  LarsonAnimation redCylonAnimation = new LarsonAnimation(255, 0, 0, 0, 0.8, ledNumber, BounceMode.Back,
      100, 0);
  LarsonAnimation blueCylonAnimation = new LarsonAnimation(0, 0, 255, 0, 0.8, ledNumber, BounceMode.Back,
      100, 0);

  public enum LightsState {
    DISABLED,
    DEFAULT,
    INTAKING,
    PLACING,
    SCORING,
    FEEDER,
    CLIMB_DEPLOY,
    CLIMB_IDLE,
    CLIMB,
  }

  public enum ItemState {
    CORAL,
    ALGAE,
    NONE,
  }

  public enum ManualState {
    MANUAL,
    AUTO,
  }

  public enum AlgaeState {
    ALGAE,
    CORAL,
  }

  public enum AllianceState {
    RED,
    BLUE,
  }

  private LightsState wantedState = LightsState.DISABLED;
  private LightsState systemState = LightsState.DISABLED;

  private AlgaeState algaeState = AlgaeState.CORAL;
  private ManualState manualState = ManualState.MANUAL;
  private ItemState itemState = ItemState.NONE;
  private AllianceState allianceState = AllianceState.RED;

  public void updateAlgaeMode(boolean algaeMode) {
    if (algaeMode) {
      algaeState = AlgaeState.ALGAE;
    } else {
      algaeState = AlgaeState.CORAL;
    }
  }

  public void updateManualMode(boolean manualMode) {
    if (manualMode) {
      manualState = ManualState.MANUAL;
    } else {
      manualState = ManualState.AUTO;
    }
  }

  public void updateIntakeItem(IntakeItem intakeItem) {
    switch (intakeItem) {
      case CORAL:
        itemState = ItemState.CORAL;
        break;
      case ALGAE:
        itemState = ItemState.ALGAE;
        break;
      default:
        itemState = ItemState.NONE;
        break;
    }
  }

  /*
   * Lights codes are as follows:
   * solid yellow - robot can't see auto chooser (might mean that robot is
   * disconected)
   * flashing yellow - error: usually means that the robot cannot see all of its
   * CAN devices and limelights
   * solid red - red alliance
   * solid blue - blue alliance
   * flashing purple:
   * autonomous - the robot does not see the note
   * intaking - robot has not intaken note yet
   * shooting - robot cannot see apriltag
   * flashing green:
   * autonomous - the robot sees the note
   * boot up/CAN check - all CAN is good and limelights are connected
   * intake - robot has intaken note
   * shooting - robot can see apriltag
   * solid green:
   * shooting - robot is aligned and ready to shoot
   */

  /**
   * Constructs a new Lights object.
   * 
   * @param tof Time of Flight sensor.
   */
  public Lights() {
  }

  public void setCommandRunning(boolean commandRunning) { // used to bypass the default light colors (red/blue)
    this.commandRunning = commandRunning;
  }

  public void setWantedState(LightsState wantedState) {
    this.wantedState = wantedState;
  }

  private LightsState handleStateTransition() {
    switch (wantedState) {
      case DEFAULT:
        return LightsState.DEFAULT;
      case INTAKING:
        return LightsState.INTAKING;
      case PLACING:
        return LightsState.PLACING;
      case SCORING:
        return LightsState.SCORING;
      case FEEDER:
        return LightsState.FEEDER;
      case CLIMB_DEPLOY:
        return LightsState.CLIMB_DEPLOY;
      case CLIMB_IDLE:
        return LightsState.CLIMB_IDLE;
      case CLIMB:
        return LightsState.CLIMB;
      default:
        return LightsState.DISABLED;
    }
  }

  /**
   * Sets the RGB values of the lights to the specified values.
   *
   * @param r The red component value (0-255).
   * @param g The green component value (0-255).
   * @param b The blue component value (0-255).
   */
  public void setCandleRGB(int r, int g, int b) { // sets the RGB values of the lights
    candleSwerve.setLEDs(r, g, b);
    candleBack.setLEDs(r, g, b);
    candleFront.setLEDs(r, g, b);
  }

  @Override
  public void periodic() {

    if (OI.isRedSide()) {
      allianceState = AllianceState.RED;
    } else {
      allianceState = AllianceState.BLUE;
    }

    LightsState newState = handleStateTransition();
    if (!DriverStation.isEnabled()) {
      newState = LightsState.DISABLED;
    }
    if (newState != systemState) {
      candleSwerve.clearAnimation(0);
      candleBack.clearAnimation(0);
      candleFront.clearAnimation(0);
      systemState = newState;
    }

    if (systemState != LightsState.DISABLED) {
      switch (manualState) {
        case MANUAL:
          setManual();
          break;
        case AUTO:
          setAuto();
          break;
        default:
          candleFront.setLEDs(10, 50, 10);
          candleSwerve.setLEDs(10, 50, 10);
          break;
      }
    }

    Logger.recordOutput("Lights State", systemState);
    switch (systemState) {
      case DEFAULT:
        switch (itemState) {
          case CORAL:
            setCoralBouncing();
            break;
          case ALGAE:
            setAlgaeBouncing();
            break;
          default:
            switch (algaeState) {
              case ALGAE:
                setAlgaeSolid();
                break;
              default:
                setCoralSolid();
                break;
            }
            break;
        }
        break;
      case INTAKING:
        switch (itemState) {
          case CORAL:
            setFlashGreen();
            break;
          case ALGAE:
            setFlashGreen();
          default:
            setFlashPurple();
            break;
        }
        break;
      case PLACING:
        setFlashYellow();
        break;
      case SCORING:
        setStrobeGreen();
        break;
      case FEEDER:
        setFlashPurple();
        break;
      case CLIMB_DEPLOY:
        setFlashPurple();
        break;
      case CLIMB_IDLE:
        setStrobeGreen();
        break;
      case CLIMB:
        setFlashYellow();
        break;
      default:
        switch (allianceState) {
          case RED:
            setRedBouncing();
            // candleSwerve.clearAnimation(0);
            // candleSwerve.setLEDs(0, 0, 0);
            break;
          default:
            setBlueBouncing();
            // candleSwerve.clearAnimation(0);
            // candleSwerve.setLEDs(0, 0, 0);
            break;
        }
        ;
    }
    // // This method will be called once per scheduler run
    // if (!commandRunning) { // only makes lights red/blue if a command is not
    // trying to change light colors
    // if (!OI.autoChooserConnected()) {
    // fieldSide = "none";
    // } else if (OI.isBlueSide()) {
    // fieldSide = "blue";
    // } else {
    // fieldSide = "red";
    // }

    // if (fieldSide == "red") { // sets lights to color of alliance
    // candle.setLEDs(255, 0, 0);
    // } else if (fieldSide == "blue") {
    // candle.setLEDs(0, 0, 255);
    // } else if (fieldSide == "none") {
    // candle.setLEDs(255, 255, 0);
    // } else {
    // candle.setLEDs(255, 255, 255);
    // }
    // } else if (timedFlashes) { // allows lights to flash for a certain period of
    // time before returning to
    // // default colors
    // if (Timer.getFPGATimestamp() - time > timeout) {
    // timedFlashes = false;
    // candle.clearAnimation(0);
    // setCommandRunning(false);
    // }
    // }
  }

  /**
   * Sets the flag indicating whether timed flashes are enabled or disabled.
   * 
   * @param timedFlashes a boolean value indicating whether timed flashes should
   *                     be enabled (true) or disabled (false).
   */
  public void setTimedFlashes(boolean timedFlashes) {
    this.timedFlashes = timedFlashes;
  }

  // public void flashGreen(double seconds) { // blinks green for a certain amount
  // of time
  // setCommandRunning(true);
  // candle0.clearAnimation(0);
  // candle1.clearAnimation(0);
  // if (seconds != -1) {
  // time = Timer.getFPGATimestamp();
  // timeout = seconds;
  // timedFlashes = true;
  // }
  // candle0.animate(greenFlash);
  // candle1.animate(greenFlash);
  // }

  // public void blinkYellow(double seconds) { // blinks yellow for a certain
  // amount of time
  // setCommandRunning(true);
  // candle0.clearAnimation(0);
  // candle1.clearAnimation(0);
  // if (seconds != -1) {
  // time = Timer.getFPGATimestamp();
  // timeout = seconds;
  // timedFlashes = true;
  // }
  // candle0.animate(yellowFlash);
  // candle1.animate(yellowFlash);
  // }

  public void clearAnimations() { // clears all animations currently running
    candleSwerve.clearAnimation(0);
    candleBack.clearAnimation(0);
    candleFront.clearAnimation(0);
  }

  public void setStrobeGreen() {
    candleSwerve.animate(greenStrobe);
    candleBack.animate(greenStrobe);
    // candleFront.animate(greenStrobe);
  }

  public void setStrobePurple() {
    candleSwerve.animate(purpleStrobe);
    candleBack.animate(purpleStrobe);
    // candleFront.animate(purpleStrobe);
  }

  public void setStrobeYellow() {
    candleSwerve.animate(yellowStrobe);
    candleBack.animate(yellowStrobe);
    // candleFront.animate(yellowStrobe);
  }

  public void setFlashGreen() {
    candleSwerve.animate(greenFlash);
    candleBack.animate(greenFlash);
    candleFront.animate(greenFlash);
  }

  public void setFlashPurple() {
    candleSwerve.animate(purpleFlash);
    candleBack.animate(purpleFlash);
    // candleFront.animate(purpleFlash);
  }

  public void setFlashYellow() {
    candleSwerve.animate(yellowFlash);
    candleBack.animate(yellowFlash);
    // candleFront.animate(yellowFlash);
  }

  public void setRedBright() {
    // candle0.animate(redSolidBright);
    // candle1.animate(redSolidBright);
    // candle2.animate(redSolidBright);
    clearAnimations();
    // setCandleRGB(255, 0, 0);
    candleBack.setLEDs(255, 0, 0);
    candleSwerve.setLEDs(255, 0, 0, 0, ledsPerSwerve * 0, ledsPerSwerve * 2);
  }

  public void setRedDim() {
    clearAnimations();
    candleBack.setLEDs(100, 0, 0);
    candleSwerve.setLEDs(100, 0, 0, 0, ledsPerSwerve * 0, ledsPerSwerve * 2);
  }

  public void setBlueBright() {
    clearAnimations();
    candleBack.setLEDs(0, 0, 255);
    candleSwerve.setLEDs(0, 0, 255, 0, ledsPerSwerve * 0, ledsPerSwerve * 2);
  }

  public void setBlueDim() {
    clearAnimations();
    candleBack.setLEDs(0, 0, 100);
    candleSwerve.setLEDs(0, 0, 100, 0, ledsPerSwerve * 0, ledsPerSwerve * 2);
  }

  public void setWhiteBright() {
    clearAnimations();
    candleBack.setLEDs(255, 255, 255);
    candleSwerve.setLEDs(255, 255, 255, 255, ledsPerSwerve * 0, ledsPerSwerve * 2);
  }

  public void setRedFlash() {
    // candleSwerve.animate(redFlash);
    candleBack.animate(redFlash);
    // candleFront.animate(redFlash);
  }

  public void setBlueFlash() {
    // candleSwerve.animate(blueFlash);
    candleBack.animate(blueFlash);
    // candleFront.animate(blueFlash);
  }

  public void setCoralSolid() {
    // candle0.animate(coralSolid);
    // candle1.animate(coralSolid);
    // candle2.animate(coralSolid);
    clearAnimations();
    candleBack.setLEDs(30, 30, 30);
    candleSwerve.setLEDs(30, 30, 30, 0, ledsPerSwerve * 0, ledsPerSwerve * 2);
  }

  public void setAlgaeSolid() {
    // candle0.animate(algaeSolid);
    // candle1.animate(algaeSolid);
    // candle2.animate(algaeSolid);
    clearAnimations();
    candleBack.setLEDs(0, 75, 25);
    candleSwerve.setLEDs(0, 75, 25, 0, ledsPerSwerve * 0, ledsPerSwerve * 2);
  }

  public void setCoralFlashing() {
    // candleSwerve.animate(coralFlashing);
    candleBack.animate(coralFlashing);
    // candleFront.animate(coralFlashing);
  }

  public void setAlgaeFlashing() {
    // candleSwerve.animate(algaeFlashing);
    candleBack.animate(algaeFlashing);
    // candleFront.animate(algaeFlashing);
  }

  public void setCoralStrobing() {
    // candleSwerve.animate(coralStrobing);
    candleBack.animate(coralStrobing);
    // candleFront.animate(coralStrobing);
  }

  public void setAlgaeStrobing() {
    // candleSwerve.animate(algaeStrobing);
    candleBack.animate(algaeStrobing);
    // candleFront.animate(algaeStrobing);
  }

  public void setCoralBouncing() {
    // candleSwerve.animate(coralKnightRiderAnimation);
    candleBack.animate(coralKnightRiderAnimation);
    // candleFront.animate(coralKnightRiderAnimation);
  }

  public void setAlgaeBouncing() {
    // candleSwerve.animate(algaeKnightRiderAnimation);
    candleBack.animate(algaeKnightRiderAnimation);
    // candleFront.animate(algaeKnightRiderAnimation);
  }

  public void setRedBouncing() {
    // candleSwerve.clearAnimation(0);
    candleSwerve.animate(redCylonAnimation);
    candleBack.animate(redCylonAnimation);
    candleFront.animate(redCylonAnimation);
  }

  public void setBlueBouncing() {
    // candleSwerve.animate(blueCylonAnimation);
    candleBack.animate(blueCylonAnimation);
    candleFront.animate(blueCylonAnimation);
  }

  public void setManual() {
    switch (allianceState) {
      case RED:
        candleFront.setLEDs(100, 0, 0);
        candleSwerve.setLEDs(100, 0, 0, 0, ledsPerSwerve * 0, ledsPerSwerve * 2);
        break;
      case BLUE:
        candleFront.setLEDs(0, 0, 100);
        candleSwerve.setLEDs(0, 0, 100, 0, ledsPerSwerve * 0, ledsPerSwerve * 2);
        break;
      default:
        candleFront.setLEDs(125, 80, 0);
        candleSwerve.setLEDs(125, 80, 0, 0, ledsPerSwerve * 0, ledsPerSwerve * 2);
        break;
    }
  }

  public void setAuto() {
    candleFront.setLEDs(80, 0, 80);
    candleSwerve.setLEDs(80, 0, 80, 0, ledsPerSwerve * 0, ledsPerSwerve * 2);
  }

  /*
   * 
   * 
   * 
   * 
   * 
   * 
   * 
   * 
   * 
   * 
   * 
   * 
   * 
   * 
   * s
   */

  /**
   * Initializes the Lights subsystem with the specified field side.
   * Clears animation at index 0 of the candle object.
   * 
   * @param fieldSide The side of the field to initialize with.
   */
  public void init(String fieldSide) {
    this.fieldSide = fieldSide;
    candleSwerve.clearAnimation(0);
    candleBack.clearAnimation(0);
    candleFront.clearAnimation(0);
  }
}