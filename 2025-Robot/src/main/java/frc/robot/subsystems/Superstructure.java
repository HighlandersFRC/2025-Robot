package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Climber.ClimbState;
import frc.robot.subsystems.Drive.DriveState;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Manipulator.ArmItem;
import frc.robot.subsystems.Manipulator.ManipulatorState;
import frc.robot.subsystems.Lights.LightsState;
import frc.robot.subsystems.Pivot.PivotFlip;
import frc.robot.subsystems.Pivot.PivotState;
import frc.robot.subsystems.Twist.TwistState;

public class Superstructure extends SubsystemBase {
  private final Drive drive;
  private final Elevator elevator;
  private final Manipulator manipulator;
  private final Pivot pivot;
  private final Twist twist;
  private final Intake intake;
  private final Climber climber;
  private final Lights lights;
  private final Peripherals peripherals;
  double outakeIdleInitTime = 0;
  boolean outakeIdleInit = false;
  boolean firstTimeDefault = true;
  private SuperState lastState = SuperState.IDLE;
  private SuperState tempLastState = SuperState.IDLE;

  public enum SuperState {
    DEFAULT,
    IDLE,
    CLIMBER_UP,
    CLIMBER_DOWN,
    GROUND_INTAKE_DOWN,
    GROUND_INTAKE_UP
  }

  private SuperState wantedSuperState = SuperState.IDLE;
  private SuperState currentSuperState = SuperState.IDLE;

  private boolean continueClimbing = false;
  public boolean algaeMode = false;
  private boolean continueFeeding = false;
  private double handoffInitTime = 0.0;

  public Superstructure(Drive drive, Elevator elevator, Manipulator manipulator, Pivot pivot, Twist twist,
      Climber climber,
      Lights lights, Peripherals peripherals, Intake intake) {
    this.drive = drive;
    this.elevator = elevator;
    this.manipulator = manipulator;
    this.pivot = pivot;
    this.twist = twist;
    this.climber = climber;
    this.lights = lights;
    this.peripherals = peripherals;
    this.intake = intake;
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

  // public boolean hasCoral() {
  //   return manipulator.hasCoral();
  // }

  // public boolean hasCoralSticky() {
  //   return manipulator.hasCoralSticky();
  // }

  private void applyStates() {
    switch (currentSuperState) {
      case DEFAULT:
        handleDefaultState();
        break;
      case CLIMBER_DOWN:
        handleClimberDown();
        break;
      case CLIMBER_UP:
        handleClimberUp();
        break;
      case GROUND_INTAKE_DOWN:
        handleGroundIntakeDown();
        break;
      case GROUND_INTAKE_UP:
        handleGroundIntakeUp();
        break;
      case IDLE:
        handleIdleState();
        break;
      default:
        handleIdleState();
        break;
    }
  }

  /**
   * This function handles the state transitions of the Superstructure subsystem.
   * It updates the current state based on the wanted state and performs necessary
   * actions.
   *
   * @return SuperState - The current state of the Superstructure subsystem after
   *         handling the state transitions.
   *
   * @param wantedSuperState The desired state of the Superstructure subsystem.
   *
   * @see SuperState
   */
  private SuperState handleStateTransitions() {
    switch (wantedSuperState) {
      case DEFAULT:
        currentSuperState = SuperState.DEFAULT;
        break;
      case CLIMBER_DOWN:
        currentSuperState = SuperState.CLIMBER_DOWN;
        break;
      case CLIMBER_UP:
        currentSuperState = SuperState.CLIMBER_UP;
        break;
      case GROUND_INTAKE_DOWN:
        currentSuperState = SuperState.GROUND_INTAKE_DOWN;
        break;
      case GROUND_INTAKE_UP:
        currentSuperState = SuperState.GROUND_INTAKE_UP;
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

  public void handleClimberUp() {
    climber.setWantedState(ClimbState.RETRACTING);
  }

  public void handleClimberDown() {
    climber.setWantedState(ClimbState.EXTENDING);
  }

  public void handleGroundIntakeDown() {
    intake.setWantedState(IntakeState.DOWN);
  }

  public void handleGroundIntakeUp() {
    intake.setWantedState(IntakeState.UP);
  }

  public void handleDefaultState() {
    lights.setWantedState(LightsState.DEFAULT);
    drive.setWantedState(DriveState.DEFAULT);
    climber.setWantedState(ClimbState.IDLE);
    elevator.setWantedState(ElevatorState.DEFAULT);
    pivot.setWantedState(PivotState.DEFAULT);
    intake.setWantedState(IntakeState.DEFAULT);
  }

  public void handleClimberIdleState() {
    climber.setWantedState(ClimbState.IDLE);
  }

  public void handleIdleState() {
    intake.setWantedState(IntakeState.IDLE);
    pivot.setWantedState(PivotState.IDLE);
    twist.setWantedState(TwistState.SIDE);
    elevator.setWantedState(ElevatorState.DEFAULT);
    drive.setWantedState(DriveState.IDLE);
    lights.setWantedState(LightsState.DEFAULT);
    climber.setWantedState(ClimbState.IDLE);
  }

  public void PARTY() {
    lights.PARTY();
  }

  @Override
  public void periodic() {
    if (climber.getTimesTriggered() && climber.getPosition() > -150) {
      PARTY();
    }

    currentSuperState = handleStateTransitions();

    if (currentSuperState != tempLastState) {
      lastState = tempLastState;
      tempLastState = currentSuperState;
    }
    if (currentSuperState != SuperState.DEFAULT) {
      firstTimeDefault = true;
    }
    Logger.recordOutput("Super State", currentSuperState);
    applyStates();
    // System.out.println("Super State: " + currentSuperState);
  }
}
