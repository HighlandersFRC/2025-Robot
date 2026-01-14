package frc.robot.subsystems.shooter;

import frc.robot.subsystems.shooter.Shooter.ShooterState;

interface ShooterIO {
    public void init();

    public void updateInputs(ShooterState systemState);

    public void setShooterPercent(double percent);

    public void setShooterRPM(double rpm);

    public void setHoodAngle(double angleDegrees);

    public double getShooterRPM();

    public double getHoodAngle();

    public double getShooterStatorCurrent();
}
