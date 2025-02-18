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
  private double strobeSpeed = 0.9;
  private double flashSpeed = 0.2;
  CANdle candle0 = new CANdle(Constants.CANInfo.CANDLE_ID_0, "rio");
  CANdle candle1 = new CANdle(Constants.CANInfo.CANDLE_ID_1, "rio");
  CANdle candle2 = new CANdle(Constants.CANInfo.CANDLE_ID_2, "rio");
  private int ledNumber = 2000;

  // ColorFlowAnimation redSolidDim = new ColorFlowAnimation(100, 0, 0, 0, 1.0, ledNumber, Direction.Forward, 0);
  // ColorFlowAnimation redSolidBright = new ColorFlowAnimation(255, 0, 0, 0, 0.0, ledNumber, Direction.Forward, 0);
  // ColorFlowAnimation blueSolidDim = new ColorFlowAnimation(0, 0, 100, 0, 0.0, ledNumber, Direction.Forward, 0);
  // ColorFlowAnimation blueSolidBright = new ColorFlowAnimation(0, 0, 255, 0, 0.0, ledNumber, Direction.Forward, 0);

  // FireAnimation redSolidDim = new FireAnimation(0.5, 1.0, ledNumber, 0.0, 0.0, false, 0);
  // LarsonAnimation redSolidDim = new LarsonAnimation(100, 0, 0, 0, 0.5, ledNumber, BounceMode.Back, 1000, 0);

  // ColorFlowAnimation greenSolidBright = new ColorFlowAnimation(0, 255, 0, 0, 0.0, ledNumber, Direction.Forward, 0);
  // ColorFlowAnimation whiteSolidBright = new ColorFlowAnimation(255, 255, 255, 255, 0.0, ledNumber, Direction.Forward,
  //     0);

  StrobeAnimation redFlash = new StrobeAnimation(255, 0, 0, 0, 0.1, ledNumber, 0);
  StrobeAnimation blueFlash = new StrobeAnimation(0, 0, 255, 0, 0.1, ledNumber, 0);

  StrobeAnimation greenStrobe = new StrobeAnimation(0, 255, 0, 0, strobeSpeed, ledNumber, 0);
  StrobeAnimation purpleStrobe = new StrobeAnimation(255, 0, 255, 0, strobeSpeed, ledNumber, 0);
  StrobeAnimation yellowStrobe = new StrobeAnimation(255, 255, 0, 0, strobeSpeed, ledNumber, 0);

  StrobeAnimation greenFlash = new StrobeAnimation(0, 255, 0, 0, flashSpeed, ledNumber, 0);
  StrobeAnimation purpleFlash = new StrobeAnimation(255, 0, 255, 0, flashSpeed, ledNumber, 0);
  StrobeAnimation yellowFlash = new StrobeAnimation(255, 255, 0, 0, flashSpeed, ledNumber, 0);

  // ColorFlowAnimation algaeSolid = new ColorFlowAnimation(0, 255, 150, 0, 0.0, ledNumber, Direction.Forward, 0);
  // ColorFlowAnimation coralSolid = new ColorFlowAnimation(255, 255, 255, 255, 0.0, ledNumber, Direction.Forward, 0);
  StrobeAnimation algaeFlashing = new StrobeAnimation(0, 255, 150, 0, flashSpeed, ledNumber, 0);
  StrobeAnimation coralFlashing = new StrobeAnimation(255, 255, 255, 255, flashSpeed, ledNumber, 0);
  StrobeAnimation algaeStrobing = new StrobeAnimation(0, 255, 150, 0, strobeSpeed, ledNumber, 0);
  StrobeAnimation coralStrobing = new StrobeAnimation(255, 255, 255, 255, strobeSpeed, ledNumber, 0);

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
    candle0.setLEDs(r, g, b);
    candle1.setLEDs(r, g, b);
    candle2.setLEDs(r, g, b);
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
      candle0.clearAnimation(0);
      candle1.clearAnimation(0);
      candle2.clearAnimation(0);
      systemState = newState;
    }
    Logger.recordOutput("Lights State", systemState);
    switch (systemState) {
      case DEFAULT:
        switch (itemState) {
          case CORAL:
            setAlgaeFlashing();
            break;
          case ALGAE:
            setAlgaeFlashing();
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
            setCoralStrobing();
            break;
          case ALGAE:
            setAlgaeStrobing();
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
            setRedDim();
            break;
          default:
            setBlueDim();
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

  // public void flashGreen(double seconds) { // blinks green for a certain amount of time
  //   setCommandRunning(true);
  //   candle0.clearAnimation(0);
  //   candle1.clearAnimation(0);
  //   if (seconds != -1) {
  //     time = Timer.getFPGATimestamp();
  //     timeout = seconds;
  //     timedFlashes = true;
  //   }
  //   candle0.animate(greenFlash);
  //   candle1.animate(greenFlash);
  // }

  // public void blinkYellow(double seconds) { // blinks yellow for a certain amount of time
  //   setCommandRunning(true);
  //   candle0.clearAnimation(0);
  //   candle1.clearAnimation(0);
  //   if (seconds != -1) {
  //     time = Timer.getFPGATimestamp();
  //     timeout = seconds;
  //     timedFlashes = true;
  //   }
  //   candle0.animate(yellowFlash);
  //   candle1.animate(yellowFlash);
  // }

  public void clearAnimations() { // clears all animations currently running
    candle0.clearAnimation(0);
    candle1.clearAnimation(0);
    candle2.clearAnimation(0);
  }

  public void setStrobeGreen() {
    candle0.animate(greenStrobe);
    candle1.animate(greenStrobe);
    candle2.animate(greenStrobe);
  }

  public void setStrobePurple() {
    candle0.animate(purpleStrobe);
    candle1.animate(purpleStrobe);
    candle2.animate(purpleStrobe);
  }

  public void setStrobeYellow() {
    candle0.animate(yellowStrobe);
    candle1.animate(yellowStrobe);
    candle2.animate(yellowStrobe);
  }

  public void setFlashGreen() {
    candle0.animate(greenFlash);
    candle1.animate(greenFlash);
    candle2.animate(greenFlash);
  }

  public void setFlashPurple() {
    candle0.animate(purpleFlash);
    candle1.animate(purpleFlash);
    candle2.animate(purpleFlash);
  }

  public void setFlashYellow() {
    candle0.animate(yellowFlash);
    candle1.animate(yellowFlash);
    candle2.animate(yellowFlash);
  }

  public void setRedBright() {
    // candle0.animate(redSolidBright);
    // candle1.animate(redSolidBright);
    // candle2.animate(redSolidBright);
    clearAnimations();
    setCandleRGB(255, 0, 0);
  }

  public void setRedDim() {
    // candle0.animate(redSolidDim);
    // candle1.animate(redSolidDim);
    // candle2.animate(redSolidDim);
    clearAnimations();
    setCandleRGB(100, 0, 0);
  }

  public void setBlueBright() {
    // candle0.animate(blueSolidBright);
    // candle1.animate(blueSolidBright);
    // candle2.animate(blueSolidBright);
    clearAnimations();
    setCandleRGB(0, 0, 255);
  }

  public void setBlueDim() {
    // candle0.animate(blueSolidDim);
    // candle1.animate(blueSolidDim);
    // candle2.animate(blueSolidDim);
    clearAnimations();
    setCandleRGB(0, 0, 100);
  }

  public void setWhiteBright() {
    // candle0.animate(whiteSolidBright);
    // candle1.animate(whiteSolidBright);
    // candle2.animate(whiteSolidBright);
    clearAnimations();
    setCandleRGB(255, 255, 255);
  }

  public void setRedFlash() {
    candle0.animate(redFlash);
    candle1.animate(redFlash);
    candle2.animate(redFlash);
  }

  public void setBlueFlash() {
    candle0.animate(blueFlash);
    candle1.animate(blueFlash);
    candle2.animate(blueFlash);
  }

  public void setCoralSolid() {
    // candle0.animate(coralSolid);
    // candle1.animate(coralSolid);
    // candle2.animate(coralSolid);
    clearAnimations();
    setCandleRGB(200, 200, 200);
  }

  public void setAlgaeSolid() {
    // candle0.animate(algaeSolid);
    // candle1.animate(algaeSolid);
    // candle2.animate(algaeSolid);
    clearAnimations();
    setCandleRGB(0, 255, 100);
  }

  public void setCoralFlashing() {
    candle0.animate(coralFlashing);
    candle1.animate(coralFlashing);
    candle2.animate(coralFlashing);
  }

  public void setAlgaeFlashing() {
    candle0.animate(algaeFlashing);
    candle1.animate(algaeFlashing);
    candle2.animate(algaeFlashing);
  }

  public void setCoralStrobing() {
    candle0.animate(coralStrobing);
    candle1.animate(coralStrobing);
    candle2.animate(coralStrobing);
  }

  public void setAlgaeStrobing() {
    candle0.animate(algaeStrobing);
    candle1.animate(algaeStrobing);
    candle2.animate(algaeStrobing);
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
    candle0.clearAnimation(0);
    candle1.clearAnimation(0);
    candle2.clearAnimation(0);
  }
}