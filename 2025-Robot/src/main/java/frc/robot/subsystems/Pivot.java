// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.apache.commons.math3.optimization.general.ConjugateGradientFormula;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SetPoints.PivotPosition;

public class Pivot extends SubsystemBase {
  /** Creates a new Pivot. */
  private final TalonFX pivotMotor = new TalonFX(Constants.CANInfo.PIVOT_MOTOR_ID,
      new CANBus(Constants.CANInfo.CANBUS_NAME));
  private final CANcoder pivotCANcoder = new CANcoder(Constants.CANInfo.PIVOT_CANCODER_ID,
      new CANBus(Constants.CANInfo.CANBUS_NAME));

  // private final PositionTorqueCurrentFOC positionTorqueFOCRequest = new
  // PositionTorqueCurrentFOC(0);

  private final double pivotJerk = 0.0;
  private final double pivotAcceleration = 7.0;
  private final double pivotCruiseVelocity = 7.0;

  private final double pivotProfileScalarFactor = 1;

  private final DynamicMotionMagicVoltage pivotMotionProfileRequest = new DynamicMotionMagicVoltage(0,
      pivotCruiseVelocity,
      pivotAcceleration,
      pivotJerk);

  public Pivot() {
  }

  public void init() {
    pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    pivotConfig.Slot0.kP = 150.0;
    pivotConfig.Slot0.kI = 0.0;
    pivotConfig.Slot0.kD = 10.0;
    pivotConfig.MotionMagic.MotionMagicJerk = this.pivotJerk;
    pivotConfig.MotionMagic.MotionMagicAcceleration = this.pivotAcceleration;
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = this.pivotCruiseVelocity;
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.StatorCurrentLimit = 60;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = 60;
    pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    pivotConfig.Feedback.FeedbackRemoteSensorID = pivotCANcoder.getDeviceID();
    pivotConfig.Feedback.SensorToMechanismRatio = 1.0;
    pivotConfig.Feedback.RotorToSensorRatio = Constants.Ratios.PIVOT_GEAR_RATIO;

    pivotMotor.getConfigurator().apply(pivotConfig);
    pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    pivotMotor.setPosition(0.0);
  }

  public void pivotToPosition(Constants.SetPoints.PivotPosition pivotPosition) {
    Logger.recordOutput("Pivot Setpoint", (pivotPosition.rotations));
    // pivotMotor
    // .setControl(positionTorqueFOCRequest
    // .withPosition((pivotPosition.rotations)));
    pivotMotor.setControl(this.pivotMotionProfileRequest
        .withPosition(pivotPosition.rotations)
        .withAcceleration(this.pivotAcceleration * pivotProfileScalarFactor)
        .withJerk(
            this.pivotJerk * pivotProfileScalarFactor));
  }

  public double getPivotPosition() {
    return (pivotMotor.getPosition().getValueAsDouble());
  }

  public void setpivotEncoderPosition(double position) {
    pivotMotor.setPosition(position);
  }

  public void setPivotPercent(double percent) {
    pivotMotor.set(percent);
  }

  public enum PivotState {
    L1,
    L23,
    L4,
    PROCESSOR,
    NET,
    FEEDER,
    GROUND_CORAL,
    GROUND_ALGAE,
    REEF_ALGAE,
    DEFAULT,
    SCORE,
  }

  private PivotState wantedState = PivotState.DEFAULT;
  private PivotState systemState = PivotState.DEFAULT;

  public void setWantedState(PivotState wantedState) {
    this.wantedState = wantedState;
  }

  private PivotState handleStateTransition() {
    switch (wantedState) {
      case DEFAULT:
        return PivotState.DEFAULT;
      case L1:
        return PivotState.L1;
      case L23:
        return PivotState.L23;
      case L4:
        return PivotState.L4;
      case FEEDER:
        return PivotState.FEEDER;
      case REEF_ALGAE:
        return PivotState.REEF_ALGAE;
      case GROUND_CORAL:
        return PivotState.GROUND_CORAL;
      case GROUND_ALGAE:
        return PivotState.GROUND_ALGAE;
      case PROCESSOR:
        return PivotState.PROCESSOR;
      case NET:
        return PivotState.NET;
      case SCORE:
        return PivotState.SCORE;
      default:
        return PivotState.DEFAULT;
    }
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Pivot Position", getPivotPosition());
    Logger.recordOutput("Pivot Output", pivotMotor.getClosedLoopOutput().getValueAsDouble());
    systemState = handleStateTransition();
    switch (systemState) {
      case DEFAULT:
        pivotToPosition(Constants.SetPoints.PivotPosition.kDEFAULT);
        break;
      case L1:
        pivotToPosition(Constants.SetPoints.PivotPosition.kUP);
        break;
      case GROUND_ALGAE:
        pivotToPosition(Constants.SetPoints.PivotPosition.kGROUNDALGAE);
        break;
      case GROUND_CORAL:
        pivotToPosition(Constants.SetPoints.PivotPosition.kGROUNDCORAL);
        break;
      default:
        setPivotPercent(0.0);
        break;
    }
    // This method will be called once per scheduler run
  }
}
