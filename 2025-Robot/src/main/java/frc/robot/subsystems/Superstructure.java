package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveState;

public class Superstructure extends SubsystemBase {
  private final Drive drive;

  public enum SuperState {
    DEFAULT,
    IDLE,
  }

  private SuperState wantedSuperState = SuperState.IDLE;
  private SuperState currentSuperState = SuperState.IDLE;

  public Superstructure(Drive drive) {
    this.drive = drive;
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
      default:
        currentSuperState = SuperState.IDLE;
        break;
    }
    return currentSuperState;

  }

  public void handleDefaultDriveState() {
    drive.setWantedState(DriveState.DEFAULT);
  }

  public void handleDefaultState() {
    drive.setWantedState(DriveState.DEFAULT);
  }

  public void handleIdleState() {
    drive.setWantedState(DriveState.IDLE);
  }

  @Override
  public void periodic() {

    currentSuperState = handleStateTransitions();
    Logger.recordOutput("Super State", currentSuperState);
    applyStates();
  }
}
