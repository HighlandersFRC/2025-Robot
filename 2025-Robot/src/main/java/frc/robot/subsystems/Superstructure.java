package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveState;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.Feeder.FeederState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;

public class Superstructure extends SubsystemBase {
  private final Drive drive;
  private final Shooter shooter;
  private final Intake intake;
  private final Feeder feeder;

  public enum SuperState {
    DEFAULT,
    IDLE,
    INTAKE,
    OUTAKE,
    PREP_SHOOT_NORMAL,
    SHOOT_NORMAL,
    PREP_SHOOT_AUTO,
    SHOOT_AUTO,
  }

  private SuperState wantedSuperState = SuperState.IDLE;
  private SuperState currentSuperState = SuperState.IDLE;

  public Superstructure(Drive drive, Shooter shooter, Intake intake, Feeder feeder) {
    this.drive = drive;
    this.shooter = shooter;
    this.intake = intake;
    this.feeder = feeder;
  }

  public void setWantedState(SuperState wantedState) {
    this.wantedSuperState = wantedState;
  }

  public Command setWantedSuperStateCommand(SuperState wantedSuperState) {
    return new InstantCommand(() -> setWantedState(wantedSuperState));
  }

  public SuperState getCurrentSuperState() {
    return currentSuperState;
  }

  private void applyStates() {
    switch (currentSuperState) {
      case DEFAULT:
        handleDefaultState();
        break;
      case IDLE:
        handleIdleState();
        break;
      case INTAKE:
        handleIntakeState();
        break;
      case OUTAKE:
        handleOutakeState();
        break;
      case SHOOT_NORMAL:
        handleShootNormalState();
        break;
      case PREP_SHOOT_NORMAL:
        handlePrepShootNormalState();
        break;
      case SHOOT_AUTO:
        handleShootAutoState();
        break;
      case PREP_SHOOT_AUTO:
        handlePrepShootAutoState();
        break;
      default:
        handleIdleState();
        break;
    }
  }

  private SuperState handleStateTransitions() {
    switch (wantedSuperState) {
      case DEFAULT:
        currentSuperState = SuperState.DEFAULT;
        break;
      case IDLE:
        currentSuperState = SuperState.IDLE;
        break;
      case INTAKE:
        currentSuperState = SuperState.INTAKE;
        break;
      case OUTAKE:
        currentSuperState = SuperState.OUTAKE;
        break;
      case SHOOT_NORMAL:
        currentSuperState = SuperState.SHOOT_NORMAL;
        break;
      case PREP_SHOOT_NORMAL:
        if (shooter.readyToShootNormal()) {
          wantedSuperState = SuperState.SHOOT_NORMAL;
          currentSuperState = SuperState.SHOOT_NORMAL;
        } else {
          currentSuperState = SuperState.PREP_SHOOT_NORMAL;
        }
        break;
      case SHOOT_AUTO:
        currentSuperState = SuperState.SHOOT_AUTO;
        break;
      case PREP_SHOOT_AUTO:
        if (shooter.readyToShootAuto()) {
          wantedSuperState = SuperState.SHOOT_AUTO;
          currentSuperState = SuperState.SHOOT_AUTO;
        } else {
          currentSuperState = SuperState.PREP_SHOOT_AUTO;
        }
        break;
      default:
        currentSuperState = SuperState.IDLE;
        break;
    }
    return currentSuperState;

  }

  public void handleDefaultState() {
    drive.setWantedState(DriveState.DEFAULT);
    shooter.setWantedState(ShooterState.IDLE);
    intake.setWantedState(IntakeState.IDLE);
    feeder.setWantedState(FeederState.IDLE);
  }

  public void handleIdleState() {
    drive.setWantedState(DriveState.DEFAULT);
    shooter.setWantedState(ShooterState.IDLE);
    intake.setWantedState(IntakeState.IDLE);
    feeder.setWantedState(FeederState.IDLE);
  }

  public void handleIntakeState() {
    drive.setWantedState(DriveState.IDLE);
    shooter.setWantedState(ShooterState.IDLE);
    intake.setWantedState(IntakeState.INTAKE);
    feeder.setWantedState(FeederState.INTAKE);
  }

  public void handleOutakeState() {
    drive.setWantedState(DriveState.DEFAULT);
    shooter.setWantedState(ShooterState.IDLE);
    intake.setWantedState(IntakeState.OUTAKE);
    feeder.setWantedState(FeederState.OUTAKE);
  }

  public void handleShootNormalState() {
    drive.setWantedState(DriveState.DEFAULT);
    shooter.setWantedState(ShooterState.SHOOTING_RPM);
    intake.setWantedState(IntakeState.IDLE);
    feeder.setWantedState(FeederState.SHOOT);
  }

  public void handlePrepShootNormalState() {
    drive.setWantedState(DriveState.DEFAULT);
    shooter.setWantedState(ShooterState.SHOOTING_RPM);
    intake.setWantedState(IntakeState.IDLE);
    feeder.setWantedState(FeederState.IDLE);
  }

  public void handleShootAutoState() {
    drive.setWantedState(DriveState.DEFAULT);
    shooter.setWantedState(ShooterState.SHOOTING_AUTO);
    intake.setWantedState(IntakeState.IDLE);
    feeder.setWantedState(FeederState.SHOOT);
  }

  public void handlePrepShootAutoState() {
    drive.setWantedState(DriveState.DEFAULT);
    shooter.setWantedState(ShooterState.SHOOTING_AUTO);
    intake.setWantedState(IntakeState.IDLE);
    feeder.setWantedState(FeederState.IDLE);
  }

  @Override
  public void periodic() {

    currentSuperState = handleStateTransitions();
    Logger.recordOutput("Super State", currentSuperState);
    applyStates();
  }
}
