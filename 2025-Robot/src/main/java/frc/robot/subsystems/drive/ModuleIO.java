package frc.robot.subsystems.drive;

public interface ModuleIO {
	/** Configure motors/encoders for a module number (PID gains, inversion, etc.) */
	void init(int moduleNumber);

	/** Angle motor closed-loop position command (rotations at the motor feedback scale) */
	void setAnglePositionRotations(double rotations);

	/** Angle motor velocity command (RPS), typically used to stop (0.0) */
	void setAngleVelocityRPS(double rps);

	/** Drive motor closed-loop velocity command (RPS at motor) */
	void setDriveVelocityRPS(double rps);

	/** Percent output for drive motor (open loop) */
	void setDrivePercent(double percent);

	/** Current limit updates for drive motor */
	void setDriveCurrentLimits(double supply, double stator);

	/** Direct encoder setters (used for seed/zeroing) */
	void setAngleMotorPositionRotations(double rotations);
	void setDriveMotorPositionRotations(double rotations);

	/** Sensor getters */
	double getAngleMotorPositionRotations();
	double getAngleMotorVelocityRPS();
	double getDriveMotorPositionRotations();
	double getDriveMotorVelocityRPS();
	double getClosedLoopRefAngle();
	double getClosedLoopRefDrive();
	double getAbsoluteEncoderRotations();
}
