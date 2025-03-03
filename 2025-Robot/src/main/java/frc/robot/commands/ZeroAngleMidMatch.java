// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;

public class ZeroAngleMidMatch extends Command {
  private Drive drive;
  private Elevator elevator;

  public ZeroAngleMidMatch(Drive drive, Elevator elevator) {
    this.drive = drive;
    this.elevator = elevator;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    drive.zeroIMU();
    elevator.setElevatorEncoderPosition(0.0);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}