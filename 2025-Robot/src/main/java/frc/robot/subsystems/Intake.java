// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final TalonFX intakeMotor = new TalonFX(Constants.CANInfo.INTAKE_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);

  private final TorqueCurrentFOC torqueCurrentFOCRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);

  public enum IntakeState {
    CORAL_INTAKE,
    ALGAE_INTAKE,
    OUTAKE,
    DEFAULT,
    OFF,
  }

  private IntakeState wantedState = IntakeState.DEFAULT;
  private IntakeState systemState = IntakeState.DEFAULT;

  public void setIntakeTorque(double current, double maxPercent) {
    intakeMotor.setControl(torqueCurrentFOCRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
  }

  public boolean hasCoral() {
    // Logger.recordOutput("Has Coral", (intakeMotor.getVelocity().getValueAsDouble() > -10
    // && intakeMotor.getTorqueCurrent().getValueAsDouble() < -15
    // && intakeMotor.getAcceleration().getValueAsDouble() < -100));
    // Logger.recordOutput("Intake Velocity", intakeMotor.getVelocity().getValueAsDouble());
    // Logger.recordOutput("Intake Torque", intakeMotor.getTorqueCurrent().getValueAsDouble());
    // Logger.recordOutput("Intake Acceleration", intakeMotor.getAcceleration().getValueAsDouble());
    if (Math.abs(intakeMotor.getVelocity().getValueAsDouble()) < 1
        && Math.abs(intakeMotor.getTorqueCurrent().getValueAsDouble()) > 8
        && Math.abs(intakeMotor.getAcceleration().getValueAsDouble()) < 10) {
      return true;
    } else {
      return false;
    }
  }

  public Intake() {
  }

  public void setIntakePercent(double percent) {
    intakeMotor.set(percent);
  }

  public double getIntakeRPS() {
    return intakeMotor.getVelocity().getValueAsDouble();
  }

  private IntakeState handleStateTransition() {
    switch (wantedState) {
      case CORAL_INTAKE:
        return IntakeState.CORAL_INTAKE;
      case ALGAE_INTAKE:
        return IntakeState.ALGAE_INTAKE;
      case OUTAKE:
        return IntakeState.OUTAKE;
      case OFF:
        return IntakeState.OFF;
      default:
        return IntakeState.DEFAULT;
    }
  }

  public void setWantedState(IntakeState wantedState) {
    this.wantedState = wantedState;
  }

  @Override
  public void periodic() {
    systemState = handleStateTransition();

    Logger.recordOutput("Intake State", systemState);
    Logger.recordOutput("Has Coral", hasCoral());
    switch (systemState) {
      case CORAL_INTAKE:
        if (hasCoral()) {
          setIntakeTorque(10, 0.2);
        } else {
          setIntakePercent(0.6);
        }
        break;
      case ALGAE_INTAKE:
        if (hasCoral()) {
          setIntakeTorque(-10, 0.2);
        } else {
          setIntakePercent(-0.7);
        }
        break;
      case OUTAKE:
        if (true/* logic for if i coral (true) or algae (false) */) {
          setIntakePercent(-0.2);
        } else {
          setIntakePercent(0.2);
        }
        break;
      case OFF:
        setIntakePercent(0.0);
        break;
      default:
        setIntakeTorque(20, 0.2);
    }
  }
}
