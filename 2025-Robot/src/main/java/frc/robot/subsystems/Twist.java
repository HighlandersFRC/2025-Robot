// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Twist extends SubsystemBase {
  /** Creates a new Twist. */
  private final TalonFX twistMotor = new TalonFX(Constants.CANInfo.TWIST_MOTOR_ID,
      new CANBus(Constants.CANInfo.CANBUS_NAME));
  // private final PositionTorqueCurrentFOC positionTorqueFOCRequest = new
  // PositionTorqueCurrentFOC(0);

  private final double twistJerk = 20.0;
  private final double twistAcceleration = 20.0;
  private final double twistCruiseVelocity = 100.0;

  private final double twistProfileScalarFactor = 3;
  private boolean startedZero = false;
  private double zeroInitTime = 0.0;

  // private final DynamicMotionMagicTorqueCurrentFOC twistMotionProfileRequest =
  // new DynamicMotionMagicTorqueCurrentFOC(0,
  // twistCruiseVelocity,
  // twistAcceleration,
  // twistJerk);
  // private final MotionMagicTorqueCurrentFOC twistMotionMagicTorqueCurrentFOC =
  // new MotionMagicTorqueCurrentFOC(0.0);

  private final MotionMagicExpoVoltage twistTorqueCurrentFOC = new MotionMagicExpoVoltage(0.0);
  private final TorqueCurrentFOC torqueCurrentFOCRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);

  public Twist() {
  }

  public void init() {
    TalonFXConfiguration twistConfig = new TalonFXConfiguration();
    twistConfig.Slot0.kP = 70.0;
    twistConfig.Slot0.kI = 0.0;
    twistConfig.Slot0.kD = 0.0;
    twistConfig.Slot0.kS = 2.0;
    twistConfig.MotionMagic.MotionMagicJerk = this.twistJerk;
    twistConfig.MotionMagic.MotionMagicAcceleration = this.twistAcceleration;
    twistConfig.MotionMagic.MotionMagicCruiseVelocity = this.twistCruiseVelocity;
    twistConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    twistConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    twistConfig.CurrentLimits.StatorCurrentLimit = 60;
    twistConfig.CurrentLimits.SupplyCurrentLimit = 60;
    twistConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    twistConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    twistTorqueCurrentFOC.EnableFOC = true;
    twistMotor.getConfigurator().apply(twistConfig);
    twistMotor.setNeutralMode(NeutralModeValue.Brake);
    twistMotor.setPosition(0.0);
  }

  public void teleopInit() {
    zeroInitTime = Timer.getFPGATimestamp();
  }

  public void twistToPosition(double rotations) {
    Logger.recordOutput("Twist Setpoint", rotations);
    // twistMotor.setControl(positionTorqueFOCRequest.withPosition(rotations *
    // Constants.Ratios.TWIST_GEAR_RATIO));
    twistMotor.setControl(this.twistTorqueCurrentFOC
        .withPosition(rotations * Constants.Ratios.TWIST_GEAR_RATIO).withEnableFOC(true));
  }

  public void setTwistPercent(double percent) {
    twistMotor.set(percent);
  }

  public void setTwistTorque(double torque, double maxPercent) {
    twistMotor.setControl(torqueCurrentFOCRequest.withOutput(torque).withMaxAbsDutyCycle(maxPercent));
  }

  public double getTwistPosition() {
    return twistMotor.getPosition().getValueAsDouble() / Constants.Ratios.TWIST_GEAR_RATIO;
  }

  public void setTwistEncoderPosition(double position) {
    twistMotor.setPosition(position);
  }

  public enum TwistState {
    UP, // idk why we have this
    SIDE, // the coral part of the intake (__)_) or (_)__) with positive X for the robot
          // being up
    DOWN, // ground intake
  }

  private TwistState wantedState = TwistState.UP;
  private TwistState systemState = TwistState.UP;

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
    Logger.recordOutput("Twist State", systemState);
    Logger.recordOutput("Twist Position Rotations", getTwistPosition());
    Logger.recordOutput("Twist Error", twistMotor.getClosedLoopError().getValueAsDouble());
    Logger.recordOutput("Twist Current", twistMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Twist MPS", (twistMotor.getVelocity().getValueAsDouble()));
    switch (systemState) {
      case UP:
        if (!startedZero) {
          zeroInitTime = Timer.getFPGATimestamp();
          startedZero = true;
        }
        if (Timer.getFPGATimestamp() - zeroInitTime > 1.0) {
          setTwistPercent(0.0);
          setTwistEncoderPosition(0.0);
        } else {
          setTwistTorque(-20, 0.2);
        }
        break;
      case SIDE:
        startedZero = false;
        zeroInitTime = 0.0;
        twistToPosition(0.25);
        break;
      case DOWN:
        startedZero = false;
        zeroInitTime = 0.0;
        twistToPosition(0.45);
        break;
      default:
        startedZero = false;
        zeroInitTime = 0.0;
        break;
    }
    // This method will be called once per scheduler run
  }
}
