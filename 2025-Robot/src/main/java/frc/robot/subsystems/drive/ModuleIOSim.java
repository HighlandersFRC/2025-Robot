package frc.robot.subsystems.drive;

public class ModuleIOSim implements ModuleIO {
    @Override
    public void init(int moduleNumber) {
        // no-op for sim
    }

    @Override
    public void setAnglePositionRotations(double rotations) {

    }

    @Override
    public void setAngleVelocityRPS(double rps) {
    }

    @Override
    public void setDriveVelocityRPS(double rps) {
    }

    @Override
    public void setDrivePercent(double percent) {
    }

    @Override
    public void setDriveCurrentLimits(double supply, double stator) {
        // no-op for sim
    }

    @Override
    public void setAngleMotorPositionRotations(double rotations) {
    }

    @Override
    public void setDriveMotorPositionRotations(double rotations) {
    }

    @Override
    public double getAngleMotorPositionRotations() {
        return 0;
    }

    @Override
    public double getAngleMotorVelocityRPS() {
        return 0;
    }

    @Override
    public double getDriveMotorPositionRotations() {
        return 0;
    }

    @Override
    public double getDriveMotorVelocityRPS() {
        return 0;
    }

    @Override
    public double getClosedLoopRefAngle() {
        return 0;
    }

    @Override
    public double getClosedLoopRefDrive() {
        return 0;
    }

    @Override
    public double getAbsoluteEncoderRotations() {
        return 0;
    }
}
