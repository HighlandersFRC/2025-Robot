// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  private final ShooterIO io;

  public Shooter() {
    if (RobotBase.isReal()) {
      io = new ShooterIOComp();
    } else {
      io = new ShooterIOSim();
    }
  }

  public void init() {
    io.init();
  }

  public void setShooterPercent(double percent) {
    io.setShooterPercent(percent);
  }

  public void setShooterRPM(double rpm) {
    io.setShooterRPM(rpm);
  }

  public void setHoodAngle(double angleDegrees) {
    io.setHoodAngle(angleDegrees);
  }

  public double getShooterRPM() {
    return io.getShooterRPM();
  }

  public double getHoodAngle() {
    return io.getHoodAngle();
  }

  public double getShooterStatorCurrent() {
    return io.getShooterStatorCurrent();
  }

  public enum ShooterState {
    IDLE,
    SHOOTING_PERCENT,
    SHOOTING_RPM,
    SHOOTING_AUTO,
  }

  public void setWantedState(ShooterState wantedState) {
    this.wantedState = wantedState;
  }

  private ShooterState wantedState = ShooterState.IDLE;
  private ShooterState systemState = ShooterState.IDLE;

  private ShooterState handleStateTransition() {
    switch (wantedState) {
      case IDLE:
        return ShooterState.IDLE;
      case SHOOTING_PERCENT:
        return ShooterState.SHOOTING_PERCENT;
      case SHOOTING_RPM:
        return ShooterState.SHOOTING_RPM;
      case SHOOTING_AUTO:
        return ShooterState.SHOOTING_AUTO;
      default:
        return ShooterState.IDLE;
    }
  }

  public boolean readyToShootNormal() {
    return (Math.abs(getShooterRPM() - 1000.0) < 100.0);
  }

  public boolean readyToShootAuto() {
    return true;
  }

  public double getShooterRPMSetpoint() {
    return 0.0;
  }

  public double getHoodAngleSetpoint() {
    return 0.0;
  }

  @Override
  public void periodic() {
    io.updateInputs(systemState);
    systemState = handleStateTransition();
    Logger.recordOutput("Shooter State", systemState);
    Logger.recordOutput("Shooter RPM", getShooterRPM());
    Logger.recordOutput("Hood Angle", getHoodAngle());
    switch (systemState) {
      case IDLE:
        setShooterPercent(0.0);
        break;
      case SHOOTING_PERCENT:
        setShooterPercent(0.5);
        setHoodAngle(10.0);
        break;
      case SHOOTING_RPM:
        setShooterRPM(1000);
        setHoodAngle(20.0);
        break;
      case SHOOTING_AUTO:
        setShooterRPM(getShooterRPMSetpoint());
        setHoodAngle(getHoodAngleSetpoint());
        break;
      default:
        setShooterPercent(0.0);
        setHoodAngle(0.0);
        break;
    }
  }
}
