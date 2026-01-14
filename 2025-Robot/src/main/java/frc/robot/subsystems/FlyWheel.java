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

public class FlyWheel extends SubsystemBase {
    /** Creates a new Intake. */
    private final TalonFX fly = new TalonFX(Constants.CANInfo.TEST_MOTOR_ID,
            Constants.CANInfo.CANBUS_NAME);
    // private final TalonFX fly2 = new TalonFX(Constants.CANInfo.TEST_MOTOR_ID2,
    // Constants.CANInfo.CANBUS_NAME);

    private final TorqueCurrentFOC m_torqueCurrentFOCRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);
    private final PositionTorqueCurrentFOC m_positionTorqueCurrentFOCRequest = new PositionTorqueCurrentFOC(0.0);
    private FlyWheelState wantedState = FlyWheelState.DEFAULT;
    private FlyWheelState systemState = FlyWheelState.DEFAULT;
    private final TorqueCurrentFOC torqueCurrentFOCRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);

    public enum FlyWheelState {
        SPINNING,
        DEFAULT,
    }

    public FlyWheel() {

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
        fly.getConfigurator().apply(config);
        fly.setNeutralMode(NeutralModeValue.Brake);
        // fly2.getConfigurator().apply(config);
        // fly2.setNeutralMode(NeutralModeValue.Brake);
    }

    private FlyWheelState handleStateTransition() {
        switch (wantedState) {
            case SPINNING:
                return FlyWheelState.SPINNING;
            default:
                return FlyWheelState.DEFAULT;
        }
    }

    public void setFlyWheelCurrent(double amps, double maxPercent) {
        fly.setControl(m_torqueCurrentFOCRequest.withOutput(amps).withMaxAbsDutyCycle(maxPercent));
        // fly2.setControl(m_torqueCurrentFOCRequest.withOutput(amps).withMaxAbsDutyCycle(maxPercent));
    }

    public void setFlyWheelPercent(double percent) {
        fly.set(percent);
        // fly2.set(-percent);
    }

    public void setWantedState(FlyWheelState wantedState) {
        this.wantedState = wantedState;
    }

    public void setFlyWheelVelocity(double rpm) {
        double rps = rpm / 60.0;
        double motorRps = rps / Constants.Ratios.TEST_MOTOR_TO_FLY_WHEEL_RATIO;
        fly.setControl(m_positionTorqueCurrentFOCRequest.withVelocity(-motorRps));
        // fly2.setControl(m_positionTorqueCurrentFOCRequest.withVelocity(motorRps));
    }

    public double getVelocity() {
        return fly.getVelocity().getValueAsDouble() * Constants.Ratios.TEST_MOTOR_TO_FLY_WHEEL_RATIO;
    }

    @Override
    public void periodic() {
        systemState = handleStateTransition();
        Logger.recordOutput("FlyWheel Stator Current", fly.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("FlyWheel Supply Current", fly.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("FlyWheel Velocity (RPM)", getVelocity() * 60.0); // convert from rps to rpm
        Logger.recordOutput("FlyWheel State", systemState);
        switch (systemState) {
            case SPINNING:
                // setFlyWheelPercent(0.55);
                setFlyWheelVelocity(2500.0);
                break;
            default:
                setFlyWheelPercent(0.0);
                break;
        }
    }

}