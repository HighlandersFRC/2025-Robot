// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  // private final TalonFX climberRoller = new TalonFX(Constants.CANInfo.CLIMBER_ROLLER_MOTOR_ID,
  //     Constants.CANInfo.CANBUS_NAME);
  private final TalonFX climberPivot = new TalonFX(Constants.CANInfo.CLIMBER_PIVOT_MOTOR_ID,
      Constants.CANInfo.CANBUS_NAME);

  // private final TorqueCurrentFOC rollerTorqueCurrentFOCRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);
  private final TorqueCurrentFOC pivotTorqueCurrentFOCRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);

  /** Creates a new Climiber. */
  public Climber() {
  }

  // public void setRollerTorque(double current, double maxPercent) {
  //   climberRoller.setControl(rollerTorqueCurrentFOCRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
  // }

  public void setPivotTorque(double current, double maxPercent) {
    System.out.println("Current: " + current + " Max Speed: " + maxPercent);
    climberPivot.setControl(pivotTorqueCurrentFOCRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
