// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Twist extends SubsystemBase {
  /** Creates a new Twist. */
  private final TalonFX twistMotor = new TalonFX(Constants.CANInfo.TWIST_MOTOR_ID,
      new CANBus(Constants.CANInfo.CANBUS_NAME));
  private final PositionTorqueCurrentFOC positionTorqueFOCRequest = new PositionTorqueCurrentFOC(0);

  public Twist() {
  }

  public void init() {
    TalonFXConfiguration twistConfig = new TalonFXConfiguration();
    twistConfig.Slot0.kP = 10.0;
    twistConfig.Slot0.kI = 0.0;
    twistConfig.Slot0.kD = 0.0;

    twistConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    twistConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    twistConfig.CurrentLimits.StatorCurrentLimit = 60;
    twistConfig.CurrentLimits.SupplyCurrentLimit = 60;
    twistConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    twistMotor.getConfigurator().apply(twistConfig);
    twistMotor.setNeutralMode(NeutralModeValue.Brake);
    twistMotor.setPosition(0.0);
  }

  public void twistToPosition(double rotations) {
    twistMotor.setControl(positionTorqueFOCRequest.withPosition(rotations*Constants.Ratios.TWIST_GEAR_RATIO));
  }

  public void setTwistPercent(double percent) {
    twistMotor.set(percent);
  }

  public double getTwistPosition() {
    return twistMotor.getPosition().getValueAsDouble()/Constants.Ratios.TWIST_GEAR_RATIO;
  }

  public void setTwistEncoderPosition(double position) {
    twistMotor.setPosition(position);
  }

  public enum TwistState {
    UP, // idk why we have this
    SIDE, // the coral part of the intake (__)_) or (_)__) with positive X for the robot being up
    DOWN, // ground intake
    
  }
  
  private TwistState wantedState = TwistState.DOWN;
  private TwistState systemState = TwistState.DOWN;

  public void setWantedState(TwistState wantedState) {
    this.wantedState = wantedState;
  }

  private TwistState handleStateTransition() {
  switch (wantedState) {
    case UP:
      return TwistState.UP;
    case SIDE:
      return TwistState.SIDE;
    case DOWN:
      return TwistState.DOWN;
    default:
      return TwistState.UP;
    }
  }

  @Override
  public void periodic() {
    systemState = handleStateTransition();
    Logger.recordOutput("Twist Position Rotations", getTwistPosition());
    Logger.recordOutput("Twist Error", twistMotor.getClosedLoopError().getValueAsDouble());
    switch (systemState) {
      case UP:
      twistToPosition(0.0);
      break;
      case SIDE:
      twistToPosition(0.25);
      break;
      case DOWN:
      // twistToPosition(0.5);
      break;
      default:
      twistToPosition(0.0);
      break;
    }
    // This method will be called once per scheduler run
  }
}
