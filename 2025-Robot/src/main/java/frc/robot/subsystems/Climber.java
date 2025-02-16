// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private final TalonFX climberRoller = new TalonFX(Constants.CANInfo.CLIMBER_ROLLER_MOTOR_ID,
      Constants.CANInfo.CANBUS_NAME);
  private final TalonFX climberPivot = new TalonFX(Constants.CANInfo.CLIMBER_PIVOT_MOTOR_ID,
      Constants.CANInfo.CANBUS_NAME);

  private final TorqueCurrentFOC rollerTorqueCurrentFOCRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);
  private final TorqueCurrentFOC pivotTorqueCurrentFOCRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);

  /** Creates a new Climiber. */
  public Climber() {
  }

  public void init() {
    climberPivot.setNeutralMode(NeutralModeValue.Brake);
  }

  public enum ClimbState {
    EXTENDING,
    RETRACTING,
    IDLE,
  }

  private ClimbState wantedState = ClimbState.IDLE;
  private ClimbState systemState = ClimbState.IDLE;

  private ClimbState handleStateTransition() {
    switch (wantedState) {
      case EXTENDING:
        return ClimbState.EXTENDING;
      case RETRACTING:
        return ClimbState.RETRACTING;
      default:
        return ClimbState.IDLE;
    }
  }

  public void setRollerTorque(double current, double maxPercent) {
    climberRoller.setControl(rollerTorqueCurrentFOCRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
  }

  public void setPivotTorque(double current, double maxPercent) {
    climberPivot.setControl(pivotTorqueCurrentFOCRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
  }

  public void setWantedState(ClimbState wantedState) {
    this.wantedState = wantedState;
  }

  @Override
  public void periodic() {
    ClimbState newState = handleStateTransition();
    if (newState != systemState) {
      systemState = newState;
    }

    Logger.recordOutput("Intake State", systemState);

    switch (systemState) {
      case EXTENDING:
        setPivotTorque(-60.0, 0.2);
        break;
      case RETRACTING:
        setPivotTorque(60.0, 0.2);
        break;
      case IDLE:
        setPivotTorque(0, 0);
        break;
      default:
        setPivotTorque(0, 0);
        break;
    }
    // This method will be called once per scheduler run
  }
}
