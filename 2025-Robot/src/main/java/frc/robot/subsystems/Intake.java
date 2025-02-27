// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final TalonFX intakeMotor = new TalonFX(Constants.CANInfo.INTAKE_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);

  private final TorqueCurrentFOC torqueCurrentFOCRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);

  private boolean algaeMode = false;
  private double timeSinceItemSwitch = 0.0;
  private double itemSwitchTime = Timer.getFPGATimestamp();
  private IntakeItem intakeItem = IntakeItem.NONE;

  public void updateAlgaeMode(boolean algaeMode) {
    this.algaeMode = algaeMode;
  }

  public enum IntakeItem {
    CORAL,
    ALGAE,
    NONE,
  }

  public void init() {
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.StatorCurrentLimit = 40;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = 40;
    intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeMotor.getConfigurator().apply(intakeConfig);
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);
  }

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
    // Logger.recordOutput("Has Coral",
    // (intakeMotor.getVelocity().getValueAsDouble() > -10
    // && intakeMotor.getTorqueCurrent().getValueAsDouble() < -15
    // && intakeMotor.getAcceleration().getValueAsDouble() < -100));
    // Logger.recordOutput("Intake Velocity",
    // intakeMotor.getVelocity().getValueAsDouble());
    // Logger.recordOutput("Intake Torque",
    // intakeMotor.getTorqueCurrent().getValueAsDouble());
    // Logger.recordOutput("Intake Acceleration",
    // intakeMotor.getAcceleration().getValueAsDouble());
    if (Math.abs(intakeMotor.getVelocity().getValueAsDouble()) < 1
        && Math.abs(intakeMotor.getTorqueCurrent().getValueAsDouble()) > 8
        && Math.abs(intakeMotor.getAcceleration().getValueAsDouble()) < 10) {
      return true;
    } else {
      return false;
    }
  }

  public IntakeItem getIntakeItem() {
    if (intakeMotor.getTorqueCurrent().getValueAsDouble() > 1.0) {
      if (Math.abs(intakeMotor.getVelocity().getValueAsDouble()) < 5.0) {
        if (Math.abs(intakeMotor.getAcceleration().getValueAsDouble()) < 10.0) {
          return IntakeItem.CORAL;
        } else {
          return IntakeItem.NONE;
        }
      } else {
        return IntakeItem.NONE;
      }
    } else if (intakeMotor.getTorqueCurrent().getValueAsDouble() < -1.0) {
      if (Math.abs(intakeMotor.getVelocity().getValueAsDouble()) < 5.0) {
        if (Math.abs(intakeMotor.getAcceleration().getValueAsDouble()) < 10.0) {
          return IntakeItem.ALGAE;
        } else {
          return IntakeItem.NONE;
        }
      } else {
        return IntakeItem.NONE;
      }
    } else {
      return IntakeItem.NONE;
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
    if (intakeItem != getIntakeItem()) {
      itemSwitchTime = Timer.getFPGATimestamp();
      intakeItem = getIntakeItem();
    }
    timeSinceItemSwitch = Timer.getFPGATimestamp() - itemSwitchTime;
    systemState = handleStateTransition();
    // System.out.println("Intake Current: " +
    // intakeMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Intake State", systemState);
    Logger.recordOutput("Intake Item", intakeItem);
    // Logger.recordOutput("Has Coral", hasCoral());
    switch (systemState) {
      case CORAL_INTAKE:
        switch (intakeItem) {
          case CORAL:
            // if (timeSinceItemSwitch > 1.0) {
            double percent = Math.hypot(OI.getDriverLeftX(), OI.getDriverLeftY()) + 0.05;
            setIntakeTorque(35, 0.1);
            // } else {
            // setIntakePercent(1.0);
            // }
            break;
          default:
            setIntakePercent(1.0);
            break;
        }
        break;
      case ALGAE_INTAKE:
        // System.out.println("algae running");
        switch (intakeItem) {
          case ALGAE:
            if (timeSinceItemSwitch > 1.0) {
              // System.out.println("intake running");
              setIntakeTorque(-40, 1.0);
            } else {
              // System.out.println("2");
              setIntakePercent(-1.0);
            }
            break;
          default:
            // System.out.println("3");
            setIntakePercent(-1.0);
            break;
        }
        break;
      case OUTAKE:
        switch (intakeItem) {
          // case CORAL:
          // setIntakePercent(-0.5);
          // break;
          // case ALGAE:
          // setIntakePercent(0.5);
          // break;
          default:
            if (algaeMode) {
              setIntakePercent(0.5);
            } else {
              setIntakePercent(-0.5);
            }
            break;
        }
        break;
      case OFF:
        setIntakePercent(0.0);
        break;
      default:
        // System.out.println("Motor Current: " + intakeMotor.getTorqueCurrent());
        if (algaeMode) {
          setIntakeTorque(-35, 0.5);
        } else {
          setIntakeTorque(25, 0.2);
        }
    }
  }
}
