// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Pivot.PivotState;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {
  /** Creates a new Pivot. */
  private final TalonFX pivotMotor = new TalonFX(Constants.CANInfo.PIVOT_MOTOR_ID,
      new CANBus(Constants.CANInfo.CANBUS_NAME));
  private final PositionTorqueCurrentFOC positionTorqueFOCRequest = new PositionTorqueCurrentFOC(0);
  
  public Pivot() {
  }

  public void init() {
    pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    pivotConfig.Slot0.kP = 1.0;
    pivotConfig.Slot0.kI = 0.0;
    pivotConfig.Slot0.kD = 0.0;

    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.StatorCurrentLimit = 60;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = 60;

    pivotMotor.getConfigurator().apply(pivotConfig);
    pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    pivotMotor.setPosition(0.0);
  }

  public void pivotToPosition(double rotations) {
    pivotMotor.setControl(positionTorqueFOCRequest.withPosition(Constants.Ratios.pivotOutRotationsToRotationsIn(rotations)));
  }

  public double getpivotPosition() {
    return Constants.Ratios.pivotInRotationsToRotationsOut(pivotMotor.getPosition().getValueAsDouble());
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
    systemState = handleStateTransition();
    switch (systemState) {
      case DEFAULT:
        pivotToPosition(0.0);
        break;
      default:
        pivotToPosition(0.0);
        break;
    }
    // This method will be called once per scheduler run
  }
}
