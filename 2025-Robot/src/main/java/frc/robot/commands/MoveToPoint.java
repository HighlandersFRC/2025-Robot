// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.DriveState;
import frc.robot.tools.math.Vector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToPoint extends Command {
  Drive drive;
  double x, y, theta;
  Boolean auto;
  double setpointTime;
  int hitSetpoint;

  /** Creates a new MoveToPoint. */
  public MoveToPoint(Drive drive, double x, double y, double theta, Boolean auto) {
    this.drive = drive;
    this.x = x;
    this.y = y;
    this.theta = theta;
    this.auto = auto;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setpointTime = 0;
    hitSetpoint = 0;
    if(auto) {
      x = drive.getReefClosestSetpoint(drive.getMT2Odometry())[0];
      y = drive.getReefClosestSetpoint(drive.getMT2Odometry())[1];
      theta = drive.getReefClosestSetpoint(drive.getMT2Odometry())[2];

      System.out.println("X: " + x);
      System.out.println("y: " + y);
      System.out.println("theta: " + theta);

      while(Math.abs(theta - drive.getMT2OdometryAngle()) > Math.PI) {
        if(theta - drive.getMT2OdometryAngle() > Math.PI) {
          theta -= 2*Math.PI;
        } else {
          theta += 2*Math.PI;
        }
      }   

    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    drive.setWantedState(DriveState.IDLE);
    drive.driveToPoint(x, y, theta);
    Logger.recordOutput("Setpoint X", x);
    Logger.recordOutput("Setpoint Y", y);
    Logger.recordOutput("Setpoint Theta", theta);
    if(auto && Math.sqrt(Math.pow((x - drive.getMT2OdometryX()), 2) + Math.pow((y - drive.getMT2OdometryY()), 2)) < 0.03 && Math.abs(theta - drive.getMT2OdometryAngle()) < 0.05) {
      hitSetpoint += 1;
    }
    if(hitSetpoint > 3 && setpointTime != 0) {
      setpointTime = Timer.getFPGATimestamp();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.autoRobotCentricDrive(new Vector(0, 0), 0);
    if(!auto) {
      drive.setWantedState(DriveState.DEFAULT);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(hitSetpoint > 3 && Timer.getFPGATimestamp() - setpointTime > 0.2) {
      return true;
    }
    

    return false;
  }
}
