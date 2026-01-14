package frc.robot.subsystems.shooter;

import frc.robot.subsystems.shooter.Shooter.ShooterState;

class ShooterIOSim implements ShooterIO {

    @Override
    public void init() {

    }

    @Override
    public void updateInputs(ShooterState systemState) {

    }

    @Override
    public void setShooterPercent(double percent) {

    }

    @Override
    public void setShooterRPM(double rpm) {

    }

    @Override
    public void setHoodAngle(double angleDegrees) {

    }

    @Override
    public double getShooterRPM() {
        return 0.0;
    }

    @Override
    public double getHoodAngle() {
        return 0.0;
    }

    @Override
    public double getShooterStatorCurrent() {
        return 0.0;
    }
}
