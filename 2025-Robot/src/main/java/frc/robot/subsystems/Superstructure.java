package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.FlyWheel.FlyWheelState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveState;

public class Superstructure extends SubsystemBase {
  private final Drive drive;
  private final FlyWheel fly;

  public enum SuperState {
    DEFAULT,
    FLY_WHEEL_TEST,
    IDLE,
  }

  private SuperState wantedSuperState = SuperState.IDLE;
  private SuperState currentSuperState = SuperState.IDLE;

  public Superstructure(Drive drive, FlyWheel fly) {
    this.drive = drive;
    this.fly = fly;
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
      case FLY_WHEEL_TEST:
        handleFlyWheelTestState();
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
      case FLY_WHEEL_TEST:
        currentSuperState = SuperState.FLY_WHEEL_TEST;
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

  public void handleFlyWheelTestState() {
    fly.setWantedState(FlyWheelState.SPINNING);
  }

  public void handleDefaultState() {
    fly.setWantedState(FlyWheelState.DEFAULT);
    drive.setWantedState(DriveState.DEFAULT);
  }

  public void handleIdleState() {
    fly.setWantedState(FlyWheelState.DEFAULT);
    drive.setWantedState(DriveState.IDLE);
  }

  @Override
  public void periodic() {

    currentSuperState = handleStateTransitions();

    applyStates();
  }
}
