package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class ModuleIOTalonFX implements ModuleIO {
	private final TalonFX angleMotor;
	private final TalonFX driveMotor;
	private final CANcoder canCoder;

	private final PositionTorqueCurrentFOC posFOC = new PositionTorqueCurrentFOC(0);
	private final VelocityTorqueCurrentFOC driveVelFOC = new VelocityTorqueCurrentFOC(0);
	private final VelocityTorqueCurrentFOC angleVelFOC = new VelocityTorqueCurrentFOC(0);

	public ModuleIOTalonFX(TalonFX angleMotor, TalonFX driveMotor, CANcoder canCoder) {
		this.angleMotor = angleMotor;
		this.driveMotor = driveMotor;
		this.canCoder = canCoder;
	}

	@Override
	public void init(int moduleNumber) {
		TalonFXConfiguration angleCfg = new TalonFXConfiguration();
		TalonFXConfiguration driveCfg = new TalonFXConfiguration();

		angleCfg.Slot0.kP = 370.0;
		angleCfg.Slot0.kI = 0.0;
		angleCfg.Slot0.kD = 15.0;

		angleCfg.Slot1.kP = 3.0;
		angleCfg.Slot1.kI = 0.0;
		angleCfg.Slot1.kD = 0.0;

		angleCfg.TorqueCurrent.PeakForwardTorqueCurrent = 70;
		angleCfg.TorqueCurrent.PeakReverseTorqueCurrent = -70;
		angleCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		angleCfg.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.1;
		angleCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		angleCfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
		angleCfg.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();
		angleCfg.Feedback.SensorToMechanismRatio = 1.0;
		angleCfg.Feedback.RotorToSensorRatio = Constants.Ratios.STEER_GEAR_RATIO;

		if (moduleNumber == 2 || moduleNumber == 3) {
			driveCfg.Slot0.kP = 9.4;
			driveCfg.Slot0.kI = 0.0;
			driveCfg.Slot0.kD = 0.0;
			driveCfg.Slot0.kV = 0.0;
		} else {
			driveCfg.Slot0.kP = 8.0;
			driveCfg.Slot0.kI = 0.0;
			driveCfg.Slot0.kD = 0.0;
			driveCfg.Slot0.kV = 0.0;
		}

		driveCfg.TorqueCurrent.PeakForwardTorqueCurrent = 120;
		driveCfg.TorqueCurrent.PeakReverseTorqueCurrent = -120;
		driveCfg.CurrentLimits.StatorCurrentLimitEnable = true;
		driveCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
		driveCfg.CurrentLimits.StatorCurrentLimit = 120;
		driveCfg.CurrentLimits.SupplyCurrentLimit = 120;

		driveCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		driveCfg.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.1;
		driveCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

		double absolutePosition = canCoder.getAbsolutePosition().getValueAsDouble();
		angleMotor.setPosition(absolutePosition);
		driveMotor.setPosition(0.0);

		angleMotor.getConfigurator().apply(angleCfg);
		driveMotor.getConfigurator().apply(driveCfg);

		angleVelFOC.Slot = 1; // used when we command velocity on angle motor (e.g., stop)
	}

	@Override
	public void setAnglePositionRotations(double rotations) {
		angleMotor.setControl(posFOC.withPosition(rotations));
	}

	@Override
	public void setAngleVelocityRPS(double rps) {
		angleMotor.setControl(angleVelFOC.withVelocity(rps));
	}

	@Override
	public void setDriveVelocityRPS(double rps) {
		driveMotor.setControl(driveVelFOC.withVelocity(rps));
	}

	@Override
	public void setDrivePercent(double percent) {
		driveMotor.set(percent);
	}

	@Override
	public void setDriveCurrentLimits(double supply, double stator) {
		CurrentLimitsConfigs cfg = new CurrentLimitsConfigs();
		cfg.StatorCurrentLimitEnable = true;
		cfg.SupplyCurrentLimitEnable = true;
		cfg.StatorCurrentLimit = stator;
		cfg.SupplyCurrentLimit = supply;
		driveMotor.getConfigurator().apply(cfg);
	}

	@Override
	public void setAngleMotorPositionRotations(double rotations) {
		angleMotor.setPosition(rotations);
	}

	@Override
	public void setDriveMotorPositionRotations(double rotations) {
		driveMotor.setPosition(rotations);
	}

	@Override
	public double getAngleMotorPositionRotations() {
		return angleMotor.getPosition().getValueAsDouble();
	}

	@Override
	public double getAngleMotorVelocityRPS() {
		return angleMotor.getVelocity().getValueAsDouble();
	}

	@Override
	public double getDriveMotorPositionRotations() {
		return driveMotor.getPosition().getValueAsDouble();
	}

	@Override
	public double getDriveMotorVelocityRPS() {
		return driveMotor.getVelocity().getValueAsDouble();
	}

	@Override
	public double getClosedLoopRefAngle() {
		return angleMotor.getClosedLoopReference().getValue();
	}

	@Override
	public double getClosedLoopRefDrive() {
		return driveMotor.getClosedLoopReference().getValue();
	}

	@Override
	public double getAbsoluteEncoderRotations() {
		return canCoder.getAbsolutePosition().getValueAsDouble();
	}
}
