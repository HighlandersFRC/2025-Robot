// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.json.JSONObject;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FeederPickup extends Command {
  Superstructure superstructure;
  private int currentPathPointIndex = 0;

  public FeederPickup(Superstructure superstructure) {
    this.superstructure = superstructure;
    addRequirements(superstructure);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public int getPathPointIndex() {
    return currentPathPointIndex;
  }

  public void from(int pointIndex, JSONObject pathJSON, int to) {
    java.util.logging.Logger.getGlobal().fine("Running Feeder in auto");
    this.currentPathPointIndex = pointIndex;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    superstructure.setWantedState(SuperState.FEEDER);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    superstructure.setWantedState(SuperState.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (superstructure.hasCoralSticky()) {
      return true;
    } else {
      return false;
    }
  }
}
