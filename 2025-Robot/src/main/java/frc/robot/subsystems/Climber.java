// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private final TalonFX climberPivot = new TalonFX(Constants.CANInfo.CLIMBER_PIVOT_MOTOR_ID,
      Constants.CANInfo.CANBUS_NAME);
  private final DigitalInput climbSensor = new DigitalInput(2);

  private final TorqueCurrentFOC rollerTorqueCurrentFOCRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);
  private final TorqueCurrentFOC pivotTorqueCurrentFOCRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);

  /** Creates a new Climiber. */
  public Climber() {
  }

  public void init() {
    TalonFXConfiguration climberConfig = new TalonFXConfiguration();
    climberConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    climberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    climberConfig.CurrentLimits.StatorCurrentLimit = 160;
    climberConfig.CurrentLimits.SupplyCurrentLimit = 160;
    climberPivot.setNeutralMode(NeutralModeValue.Brake);
    climberPivot.setPosition(0.0);

  }

  public double getPosition() {
    return climberPivot.getPosition().getValueAsDouble();
  }

  public enum ClimbState {
    EXTENDING,
    RETRACTING,
    IDLE,
    DEFAULT,
  }

  private ClimbState wantedState = ClimbState.DEFAULT;
  private ClimbState systemState = ClimbState.DEFAULT;

  private ClimbState handleStateTransition() {
    switch (wantedState) {
      case EXTENDING:
        return ClimbState.EXTENDING;
      case RETRACTING:
        return ClimbState.RETRACTING;
      case IDLE:
        return ClimbState.IDLE;
      default:
        return ClimbState.DEFAULT;
    }
  }

  public void setPivotTorque(double current, double maxPercent) {
    climberPivot.setControl(pivotTorqueCurrentFOCRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
  }

  public void setWantedState(ClimbState wantedState) {
    this.wantedState = wantedState;
  }

  public boolean getClimbSensor() {
    return climbSensor.get();
  }

  @Override
  public void periodic() {
    // if ((" " + Timer.getFPGATimestamp()).indexOf("0") > 6) {
    //   System.out.println(getClimbSensor());
    // }
    Logger.recordOutput("Climb Sensor", getClimbSensor());
    ClimbState newState = handleStateTransition();
    if (newState != systemState) {
      systemState = newState;
    }

    Logger.recordOutput("Climber State", systemState);
    switch (systemState) {
      case EXTENDING:
        setPivotTorque(-160.0, 1.0);
        break;
      case RETRACTING:
        setPivotTorque(160.0, 1.0);
        break;
      case IDLE:
        setPivotTorque(0, 0);
        break;
      default:
        // setPivotTorque(0, 0);
        break;
    }
    // This method will be called once per scheduler run
  }
}
