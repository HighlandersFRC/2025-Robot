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
import frc.robot.subsystems.intake.Intake.IntakeState;

public class Roller extends SubsystemBase {
    /** Creates a new Intake. */
    private final TalonFX roller = new TalonFX(30,
            Constants.CANInfo.CANBUS_NAME);
    // private final TalonFX roller2 = new TalonFX(Constants.CANInfo.TEST_MOTOR_ID2,
    // Constants.CANInfo.CANBUS_NAME);

    private final TorqueCurrentFOC m_torqueCurrentFOCRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);
    private final PositionTorqueCurrentFOC m_positionTorqueCurrentFOCRequest = new PositionTorqueCurrentFOC(0.0);
    private RollerState wantedState = RollerState.DEFAULT;
    private RollerState systemState = RollerState.DEFAULT;
    private final TorqueCurrentFOC torqueCurrentFOCRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);

    public enum RollerState {
        SPINNING,
        DEFAULT,
    }

    public Roller() {

    }

    public void init() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 120;
        config.CurrentLimits.SupplyCurrentLimit = 120;
        config.Slot0.kP = 400.068;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0;
        // config.Slot0.kG = 0;
        config.MotionMagic.MotionMagicAcceleration = Constants.SetPoints.IntakeSetpoints.INTAKE_ACCELERATION;
        config.MotionMagic.MotionMagicCruiseVelocity = Constants.SetPoints.IntakeSetpoints.INTAKE_CRUISE_VELOCITY;
        roller.getConfigurator().apply(config);
        roller.setNeutralMode(NeutralModeValue.Brake);
        // roller2.getConfigurator().apply(config);
        // roller2.setNeutralMode(NeutralModeValue.Brake);
    }

    private RollerState handleStateTransition() {
        switch (wantedState) {
            case SPINNING:
                return RollerState.SPINNING;
            default:
                return RollerState.DEFAULT;
        }
    }

    public void setRollerCurrent(double amps, double maxPercent) {
        roller.setControl(m_torqueCurrentFOCRequest.withOutput(amps).withMaxAbsDutyCycle(maxPercent));
        // roller2.setControl(m_torqueCurrentFOCRequest.withOutput(amps).withMaxAbsDutyCycle(maxPercent));
    }

    public void setRollerPercent(double percent) {
        roller.set(percent);
        // roller2.set(-percent);
    }

    public void setWantedState(RollerState wantedState) {
        this.wantedState = wantedState;
    }

    public void setRollerVelocity(double rpm) {
        double rps = rpm / 60.0;
        double motorRps = rps / Constants.Ratios.TEST_MOTOR_TO_FLY_WHEEL_RATIO;
        roller.setControl(m_positionTorqueCurrentFOCRequest.withVelocity(-motorRps));
        // roller2.setControl(m_positionTorqueCurrentFOCRequest.withVelocity(motorRps));
    }

    public double getVelocity() {
        return roller.getVelocity().getValueAsDouble() * Constants.Ratios.TEST_MOTOR_TO_FLY_WHEEL_RATIO;
    }

    @Override
    public void periodic() {
        systemState = handleStateTransition();
        Logger.recordOutput("Roller Stator Current", roller.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Roller Supply Current", roller.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Roller Velocity (RPM)", getVelocity() * 60.0); // convert from rps to rpm
        Logger.recordOutput("Roller State", systemState);
        switch (systemState) {
            case SPINNING:
                // setRollerPercent(0.55);
                setRollerVelocity(2500.0);
                break;
            default:
                // setRollerPercent(0.0);
                break;
        }
    }

}