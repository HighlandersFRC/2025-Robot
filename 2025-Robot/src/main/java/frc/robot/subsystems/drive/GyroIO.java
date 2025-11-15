package frc.robot.subsystems.drive;

import frc.robot.tools.math.Vector;

public interface GyroIO {
	/** Apply configs and zero as needed. */
	void init();

	/** Zero yaw to 0° */
	void zeroYaw();

	/** Set yaw in degrees */
	void setYaw(double degrees);

	/** Get yaw in degrees */
	double getYawDegrees();

	/** Convenience getter in radians */
	default double getYawRadians() {
		return Math.toRadians(getYawDegrees());
	}

	/** Secondary IMU yaw (if present), in degrees */
	double getSecondaryYawDegrees();

	/** Z angular velocity in world coordinates, radians/s */
	double getAngularVelocityZWorldRadPerSec();

	/** Z angular velocity in device coordinates, degrees/s */
	double getAngularVelocityZDeviceDegPerSec();

	/** Pitch in degrees (device-reported) */
	double getPitchDegrees();

	/** Pitch adjusted by current offset */
	double getPitchAdjustedDegrees();

	/** Set pitch offset (degrees) to subtract from raw pitch when adjusted */
	void setPitchOffsetDegrees(double offsetDeg);

	/** Linear acceleration vector normalized by g (robot-centric) */
	Vector getLinearAccelGVector();
}
