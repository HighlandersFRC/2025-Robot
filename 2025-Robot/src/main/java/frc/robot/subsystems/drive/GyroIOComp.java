package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.Constants;
import frc.robot.tools.math.Vector;

public class GyroIOComp implements GyroIO {
    private final Pigeon2 pigeon = new Pigeon2(0, "Canivore");
    private final Pigeon2 pigeonExtra = new Pigeon2(1, "Canivore");

    private final Pigeon2Configuration pigeonConfig = new Pigeon2Configuration();
    private final Pigeon2Configuration pigeonExtraConfig = new Pigeon2Configuration();

    private double pitchOffset = 0.0;

    @Override
    public void init() {
        pigeonConfig.MountPose.MountPosePitch = 0.3561480641365051;
        pigeonConfig.MountPose.MountPoseRoll = -0.10366992652416229;
        pigeonConfig.MountPose.MountPoseYaw = -0.24523599445819855;

        pigeonExtraConfig.MountPose.MountPosePitch = 2.9378318786621094;
        pigeonExtraConfig.MountPose.MountPoseRoll = -1.7237101793289185;
        pigeonExtraConfig.MountPose.MountPoseYaw = -1.0769075155258179;

        pigeon.getConfigurator().apply(pigeonConfig);
        pigeonExtra.getConfigurator().apply(pigeonExtraConfig);

        zeroYaw();
        setPitchOffsetDegrees(getPitchDegrees());
    }

    @Override
    public void zeroYaw() {
        setYaw(0.0);
    }

    @Override
    public void setYaw(double degrees) {
        pigeon.setYaw(degrees);
        pigeonExtra.setYaw(degrees);
    }

    @Override
    public double getYawDegrees() {
        return pigeon.getYaw().getValueAsDouble();
    }

    @Override
    public double getSecondaryYawDegrees() {
        return pigeonExtra.getYaw().getValueAsDouble();
    }

    @Override
    public double getAngularVelocityZWorldRadPerSec() {
        return pigeon.getAngularVelocityZWorld().getValueAsDouble();
    }

    @Override
    public double getAngularVelocityZDeviceDegPerSec() {
        return Math.abs(pigeon.getAngularVelocityZDevice().getValueAsDouble());
    }

    @Override
    public double getPitchDegrees() {
        return pigeon.getPitch().getValueAsDouble();
    }

    @Override
    public double getPitchAdjustedDegrees() {
        return getPitchDegrees() - pitchOffset;
    }

    @Override
    public void setPitchOffsetDegrees(double offsetDeg) {
        pitchOffset = offsetDeg;
    }

    @Override
    public Vector getLinearAccelGVector() {
        Vector v = new Vector();
        v.setI(pigeon.getAccelerationX().getValueAsDouble() / Constants.Physical.GRAVITY_ACCEL_MS2);
        v.setJ(pigeon.getAccelerationY().getValueAsDouble() / Constants.Physical.GRAVITY_ACCEL_MS2);
        return v;
    }
}
