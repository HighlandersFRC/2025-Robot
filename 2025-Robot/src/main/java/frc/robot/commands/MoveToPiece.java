// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.DriveState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Peripherals;
import frc.robot.tools.controlloops.PID;
import frc.robot.tools.math.Vector;

public class MoveToPiece extends Command {
  Drive drive;
  Peripherals peripherals;
  Intake intake;

  private PID pid;
  private double kP = 3;
  private double kI = 0;
  private double kD = 0;
  private double[] desiredVelocityArray = new double[3];
  private double desiredThetaChange = 0;

  private double ty, tx;
  private double noteX, noteY;

  /** Creates a new MoveToPiece. */
  public MoveToPiece(Drive drive, Peripherals peripherals, Intake intake) {
    this.drive = drive;
    this.peripherals = peripherals;
    this.intake = intake;
    addRequirements(this.drive, this.intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid = new PID(kP, kI, kD);
    pid.setSetPoint(0);
    pid.setMinOutput(-4);
    pid.setMaxOutput(4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.setWantedState(DriveState.IDLE);

    tx = peripherals.getGamePieceCamYaw();
    ty = peripherals.getGamePiecePitch();

    double limelightHeight = Constants.inchesToMeters(34.75);
    double limelightAngle = 33.0;
    // double limelightXOffset = -Constants.inchesToMeters(4.0);
    double gamePieceHeight = Constants.inchesToMeters(7.5);
    double intakeXOffset = Constants.inchesToMeters(26.5);
    double intakeYOffset = Constants.inchesToMeters(3.0);

    double robotX = drive.getFusedOdometryX();
    double robotY = drive.getFusedOdometryY();
    double robotAngle = Constants.standardizeAngleDegrees(peripherals.getPigeonAngle());

    double targetDistance = (limelightHeight - gamePieceHeight) / Math.tan(Math.toRadians(-limelightAngle + ty));

    noteY = robotY + (targetDistance * Math.sin(Math.toRadians(tx)));
    noteX = robotX + (-(targetDistance * Math.cos(Math.toRadians(tx))));
    double pieceXFromIntake = noteX - intakeXOffset;
    double pieceYFromIntake = noteY - intakeYOffset;

    Logger.recordOutput("Piece X", Constants.metersToInches(pieceXFromIntake));
    Logger.recordOutput("Piece Y", Constants.metersToInches(pieceYFromIntake));

    // drive.driveToPoint(noteX,
    // noteY, Math.toRadians(-tx + 1.6));
    drive.teleopDriveToPiece(-pieceYFromIntake);
    if (Math.abs(intake.getIntakeRPS()) > 10) {
      intake.setIntakeTorque(-20, 0.7);
    } else {
      intake.setIntakePercent(-0.7);
    }

    // Vector velocityVector = new Vector();
    // velocityVector.setI(desiredVelocityArray[0]);
    // velocityVector.setJ(desiredVelocityArray[1]);
    // desiredThetaChange = desiredVelocityArray[2];
    // velocityVector.setI(0);
    // velocityVector.setJ(0);
    // desiredThetaChange = 0;

    // drive.autoDrive(velocityVector, desiredThetaChange);

    // double angleToPiece = peripherals.getBackCamTargetTx();
    // pid.updatePID(angleToPiece);
    // double result = -pid.getResult();

    // drive.autoRobotCentricDrive(new Vector(-3, 0), result);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Vector velocityVector = new Vector();
    // velocityVector.setI(0);
    // velocityVector.setJ(0);
    // double desiredThetaChange = 0.0;
    // drive.autoDrive(velocityVector, desiredThetaChange);
    drive.setWantedState(DriveState.DEFAULT);
    intake.setIntakeTorque(-10, 0.2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (!intake.getBeamBreak()) {
    // return true;
    // } else {
    return false;
    // }
  }
}