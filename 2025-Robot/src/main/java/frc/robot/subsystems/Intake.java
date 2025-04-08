// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.tools.BeamBreak;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final TalonFX roller = new TalonFX(Constants.CANInfo.INTAKE_ROLLER_MOTOR_ID,
      Constants.CANInfo.CANBUS_NAME);
  private final TalonFX pivot = new TalonFX(Constants.CANInfo.INTAKE_PIVOT_MOTOR_ID,
      Constants.CANInfo.CANBUS_NAME);
  private final BeamBreak beamBreak = new BeamBreak(Constants.CANInfo.INTAKE_BEAM_BREAK_PORT);
  private final TorqueCurrentFOC m_torqueCurrentFOCRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);
  private final PositionTorqueCurrentFOC m_positionTorqueCurrentFOCRequest = new PositionTorqueCurrentFOC(0.0);
  private IntakeState wantedState = IntakeState.DEFAULT;
  private IntakeState systemState = IntakeState.DEFAULT;
  private boolean isZeroed = false;
  private final TorqueCurrentFOC torqueCurrentFOCRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);
  private double handOffTime = Timer.getFPGATimestamp();
  private boolean firstTimeHandOff = true;

  public enum IntakeState {
    INTAKING,
    OUTAKING,
    HANDOFF,
    IDLE,
    DEFAULT,
    ZERO,
    L4,
  }

  public Intake() {

  }

  public void init() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.Slot0.kP = 4;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    config.Slot0.kG = 0;
    config.MotionMagic.MotionMagicAcceleration = Constants.SetPoints.IntakeSetpoints.INTAKE_ACCELERATION;
    config.MotionMagic.MotionMagicCruiseVelocity = Constants.SetPoints.IntakeSetpoints.INTAKE_CRUISE_VELOCITY;
    roller.getConfigurator().apply(config);
    roller.setNeutralMode(NeutralModeValue.Brake);
    pivot.getConfigurator().apply(config);
    pivot.setNeutralMode(NeutralModeValue.Brake);
    pivot.setPosition(0);
  }

  private IntakeState handleStateTransition() {
    switch (wantedState) {
      case INTAKING:
        return IntakeState.INTAKING;
      case ZERO:
        return IntakeState.ZERO;
      case OUTAKING:
        return IntakeState.OUTAKING;
      case IDLE:
        return IntakeState.IDLE;
      case DEFAULT:
        return IntakeState.DEFAULT;
      case HANDOFF:
        return IntakeState.HANDOFF;
      case L4:
        return IntakeState.L4;
      default:
        return IntakeState.IDLE;
    }
  }

  public void pivotWithTorque(double current, double maxPercent) {
    pivot.setControl(torqueCurrentFOCRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
  }

  public void pivotToPosition(double pivotRotations) {
    pivot.setControl(m_positionTorqueCurrentFOCRequest
        .withPosition(pivotRotations * Constants.Ratios.INTAKE_PIVOT_GEAR_RATIO)
        .withVelocity(Constants.SetPoints.IntakeSetpoints.INTAKE_CRUISE_VELOCITY
            * Constants.SetPoints.IntakeSetpoints.INTAKE_MOTION_PROFILE_SCALAR)
        .withSlot(0));
  }

  public void setRollerCurrent(double amps, double maxPercent) {
    roller.setControl(m_torqueCurrentFOCRequest.withOutput(amps).withMaxAbsDutyCycle(maxPercent));
  }

  public void setRollerPercent(double percent) {
    roller.set(percent);
  }

  public void setWantedState(IntakeState wantedState) {
    this.wantedState = wantedState;
  }

  public double getPosition() {
    return pivot.getPosition().getValueAsDouble() / Constants.Ratios.INTAKE_PIVOT_GEAR_RATIO;
  }

  public boolean getZeroed() {
    if (Math.abs(pivot.getStatorCurrent().getValueAsDouble()) > 10.0
        && Math.abs(pivot.getVelocity().getValueAsDouble()) < 5.0) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Intake Motor Current", roller.getStatorCurrent().getValueAsDouble());
    systemState = handleStateTransition();
    if (systemState != IntakeState.HANDOFF) {
      firstTimeHandOff = true;
    }
    // System.out.println("Intake Current: " +
    // intakeMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Intake State", systemState);
    Logger.recordOutput("Intake Has coral", hasCoral());
    // Logger.recordOutput("Has Coral", hasCoral());
    switch (systemState) {
      case INTAKING:
        pivotToPosition(Constants.SetPoints.IntakeSetpoints.INTAKE_DOWN);
        setRollerCurrent(Constants.SetPoints.IntakeSetpoints.INTAKE_ROLLER_TORQUE,
            Constants.SetPoints.IntakeSetpoints.INTAKE_ROLLER_MAX_SPEED);
        break;
      case L4:
        pivotToPosition(Constants.SetPoints.IntakeSetpoints.INTAKE_DOWN);
        setRollerCurrent(Constants.SetPoints.IntakeSetpoints.INTAKE_ROLLER_TORQUE,
            0.5);
        break;
      case ZERO:
        setRollerCurrent(Constants.SetPoints.IntakeSetpoints.INTAKE_ROLLER_TORQUE,
            Constants.SetPoints.IntakeSetpoints.INTAKE_ROLLER_HOLDING_SPEED);
        pivotWithTorque(-20, 0.5);
        if (getZeroed()) {
          pivot.setPosition(0.0);
        }
        break;
      case OUTAKING:
        // setRollerCurrent(-Constants.SetPoints.IntakeSetpoints.INTAKE_ROLLER_TORQUE,
        //     Constants.SetPoints.IntakeSetpoints.INTAKE_ROLLER_MAX_SPEED);
        setRollerPercent(-1.0);
        pivotToPosition(Constants.SetPoints.IntakeSetpoints.INTAKE_DOWN);
        break;
      case HANDOFF:
        if (firstTimeHandOff) {
          firstTimeHandOff = false;
          handOffTime = Timer.getFPGATimestamp();
        }
        // if (Timer.getFPGATimestamp() - handOffTime < 3.0) {
        // setRollerCurrent(-Constants.SetPoints.IntakeSetpoints.INTAKE_ROLLER_TORQUE,
        //     Constants.SetPoints.IntakeSetpoints.INTAKE_ROLLER_MAX_SPEED);
        setRollerPercent(-1.0);
        pivotToPosition(Constants.SetPoints.IntakeSetpoints.INTAKE_UP);
        // } else {
        //   pivotToPosition(Constants.SetPoints.IntakeSetpoints.INTAKE_UP);
        //   setRollerCurrent(-Constants.SetPoints.IntakeSetpoints.INTAKE_ROLLER_TORQUE,
        //       Constants.SetPoints.IntakeSetpoints.INTAKE_ROLLER_HOLDING_SPEED);

        // }
        break;
      case IDLE:
        pivotToPosition(Constants.SetPoints.IntakeSetpoints.INTAKE_UP);
        roller.set(0);
        break;
      case DEFAULT:
        if (Math.abs(getPosition() - Constants.SetPoints.IntakeSetpoints.INTAKE_UP) < 0.1) {
          pivotWithTorque(-20, 0.2);
          setRollerCurrent(Constants.SetPoints.IntakeSetpoints.INTAKE_HOLDING_TORQUE,
              0.1);
        } else {
          pivotToPosition(Constants.SetPoints.IntakeSetpoints.INTAKE_UP);
          setRollerCurrent(Constants.SetPoints.IntakeSetpoints.INTAKE_HOLDING_TORQUE,
              0.1);
        }
        break;
      default:
        if (Math.abs(pivot.getVelocity().getValueAsDouble()) < 0.01 && !isZeroed) {
          this.setPivotCurrent(-10, 0.1);
          pivot.setPosition(0);
        } else if (!isZeroed) {
          this.setPivotCurrent(-45, 0.3);
        } else {
          this.setPivotCurrent(-5, 0.1);
        }
        if (Math.abs(pivot.getPosition().getValueAsDouble()) > 2) {
          isZeroed = false;
        }
    }
  }

  private void setPivotCurrent(double amps, double maxPercent) {
    pivot.setControl(m_torqueCurrentFOCRequest.withOutput(amps).withMaxAbsDutyCycle(maxPercent));
  }

  private boolean lastCoralValue = false;
  private double switchTime = Timer.getFPGATimestamp();
  private boolean hasCoralSticky = false;

  public boolean hasCoral() {
    // Logger.recordOutput("Intake Velocity",
    //     roller.getVelocity().getValueAsDouble());
    // Logger.recordOutput("Intake Torque",
    //     roller.getTorqueCurrent().getValueAsDouble());
    // Logger.recordOutput("Intake Acceleration",
    //     roller.getAcceleration().getValueAsDouble());
    // Logger.recordOutput("Intake Position", getPosition());
    if (Math.abs(roller.getVelocity().getValueAsDouble()) < 5
        && Math.abs(roller.getTorqueCurrent().getValueAsDouble()) > 20
    /* && Math.abs(roller.getAcceleration().getValueAsDouble()) < 10 */
    // && beamBreak.isTripped()
    ) {
      if (lastCoralValue != true) {
        switchTime = Timer.getFPGATimestamp();
        System.out.println("Switch Ground Intake Item: Has Coral");
      }
      lastCoralValue = true;
      return true;
    } else {
      if (lastCoralValue != false) {
        switchTime = Timer.getFPGATimestamp();
        System.out.println("Switch Ground Intake Item: Empty");
      }
      lastCoralValue = false;
      return false;
    }
  }

  public boolean hasCoralSuperSticky() {
    if (hasCoral() && Timer.getFPGATimestamp() - switchTime > 0.05) {
      hasCoralSticky = true;
    } else if (!hasCoral() && Timer.getFPGATimestamp() - switchTime > 1.5) {
      hasCoralSticky = false;
    }
    return hasCoralSticky;
  }

}
