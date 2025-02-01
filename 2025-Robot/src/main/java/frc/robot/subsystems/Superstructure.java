package frc.robot.subsystems;

import org.apache.commons.math3.optim.linear.PivotSelectionRule;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive.DriveState;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Pivot.PivotFlip;
import frc.robot.subsystems.Pivot.PivotState;
import frc.robot.subsystems.Twist.TwistState;

public class Superstructure extends SubsystemBase {
  private Drive drive;
  private Elevator elevator;
  private Intake intake;
  private Pivot pivot;
  private Twist twist;

  public enum SuperState {
    DEFAULT,
    AUTO_L1_PLACE,
    AUTO_L2_PLACE,
    AUTO_L3_PLACE,
    AUTO_L4_PLACE,
    L1_PLACE,
    L2_PLACE,
    L3_PLACE,
    L4_PLACE,
    PROCESSOR,
    NET,
    FEEDER,
    GROUND_CORAL_PICKUP_FRONT,
    GROUND_CORAL_PICKUP_BACK,
    GROUND_ALGAE_PICKUP,
    L2_ALGAE_PICKUP,
    L3_ALGAE_PICKUP,
    DEPLOY_CLIMBER,
    CLIMB,
    OUTAKE,
    SCORE_L1,
    SCORE_L2,
    SCORE_L3,
    SCORE_L4,
    AUTO_SCORE_L1,
    AUTO_SCORE_L2,
    AUTO_SCORE_L3,
    AUTO_SCORE_L4,
    IDLE,
  }

  private SuperState wantedSuperState = SuperState.DEFAULT;
  private SuperState currentSuperState = SuperState.DEFAULT;

  public Superstructure(Drive drive, Elevator elevator, Intake intake, Pivot pivot, Twist twist) {
    this.drive = drive;
    this.elevator = elevator;
    this.intake = intake;
    this.pivot = pivot;
    this.twist = twist;
  }

  public void setWantedState(SuperState wantedState) {
    this.wantedSuperState = wantedState;
  }

  public Command setWantedSuperStateCommand(SuperState wantedSuperState) {
    return new InstantCommand(() -> setWantedState(wantedSuperState));
  }

  private void applyStates() {
    switch (currentSuperState) {
      case DEFAULT:
        handleDefaultState();
        break;
      case AUTO_L1_PLACE:
        handleAutoL1PlaceState();
        break;
      case AUTO_L2_PLACE:
        handleAutoL2PlaceState();
        break;
      case AUTO_L3_PLACE:
        handleAutoL3PlaceState();
        break;
      case AUTO_L4_PLACE:
        handleAutoL4PlaceState();
        break;
      case L1_PLACE:
        handleL1PlaceState();
        break;
      case L2_PLACE:
        handleL2PlaceState();
        break;
      case L3_PLACE:
        handleL3PlaceState();
        break;
      case L4_PLACE:
        handleL4PlaceState();
        break;
      case PROCESSOR:
        handleProcessorState();
        break;
      case NET:
        handleNetState();
        break;
      case FEEDER:
        handleFeederState();
        break;
      case GROUND_CORAL_PICKUP_FRONT:
        handleGroundCoralPickupFrontState();
        break;
      case GROUND_CORAL_PICKUP_BACK:
        handleGroundCoralPickupBackState();
        break;
      case GROUND_ALGAE_PICKUP:
        handleGroundAlgaePickupState();
        break;
      case L2_ALGAE_PICKUP:
        handleL2AlgaePickupState();
        break;
      case L3_ALGAE_PICKUP:
        handleL3AlgaePickupState();
        break;
      case DEPLOY_CLIMBER:
        handleDeployClimberState();
        break;
      case CLIMB:
        handleClimbState();
        break;
      case OUTAKE:
        handleOutakeState();
        break;
      case SCORE_L1:
        handleScoreL1State();
        break;
      case SCORE_L2:
        handleScoreL2State();
        break;
      case SCORE_L3:
        handleScoreL3State();
        break;
      case SCORE_L4:
        handleScoreL4State();
        break;
      case AUTO_SCORE_L1:
        handleAutoL1ScoreState();
        break;
      case AUTO_SCORE_L2:
        handleAutoL2ScoreState();
        break;
      case AUTO_SCORE_L3:
        handleAutoL3ScoreState();
        break;
      case AUTO_SCORE_L4:
        handleAutoL4ScoreState();
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
      case AUTO_L1_PLACE:
        currentSuperState = SuperState.AUTO_L1_PLACE;
        break;
      case AUTO_L2_PLACE:
        if (drive.hitSetPoint() && elevator.getElevatorPosition() > 15 / 39.37) {
          currentSuperState = SuperState.AUTO_SCORE_L2;
          wantedSuperState = SuperState.AUTO_SCORE_L2;
        } else {
          currentSuperState = SuperState.AUTO_L2_PLACE;
        }
        break;
      case AUTO_L3_PLACE:
        if (drive.hitSetPoint() && elevator.getElevatorPosition() > 32 / 39.37) {
          currentSuperState = SuperState.AUTO_SCORE_L3;
          wantedSuperState = SuperState.AUTO_SCORE_L3;
        } else {
          currentSuperState = SuperState.AUTO_L3_PLACE;
        }
        break;
      case AUTO_L4_PLACE:
        if (drive.hitSetPoint() && elevator.getElevatorPosition() > 53 / 39.37) {
          currentSuperState = SuperState.AUTO_SCORE_L4;
          wantedSuperState = SuperState.AUTO_SCORE_L4;
        } else {
          currentSuperState = SuperState.AUTO_L4_PLACE;
        }
        break;
      case L1_PLACE:
        currentSuperState = SuperState.L1_PLACE;
        break;
      case L2_PLACE:
        currentSuperState = SuperState.L2_PLACE;
        break;
      case L3_PLACE:
        currentSuperState = SuperState.L3_PLACE;
        break;
      case L4_PLACE:
        currentSuperState = SuperState.L4_PLACE;
        break;
      case PROCESSOR:
        currentSuperState = SuperState.PROCESSOR;
        break;
      case NET:
        currentSuperState = SuperState.NET;
        break;
      case FEEDER:
        currentSuperState = SuperState.FEEDER;
        break;
      case GROUND_CORAL_PICKUP_FRONT:
        currentSuperState = SuperState.GROUND_CORAL_PICKUP_FRONT;
        break;
      case GROUND_CORAL_PICKUP_BACK:
        currentSuperState = SuperState.GROUND_CORAL_PICKUP_BACK;
        break;
      case GROUND_ALGAE_PICKUP:
        currentSuperState = SuperState.GROUND_ALGAE_PICKUP;
        break;
      case L2_ALGAE_PICKUP:
        currentSuperState = SuperState.L2_ALGAE_PICKUP;
        break;
      case L3_ALGAE_PICKUP:
        currentSuperState = SuperState.L3_ALGAE_PICKUP;
        break;
      case DEPLOY_CLIMBER:
        currentSuperState = SuperState.DEPLOY_CLIMBER;
        break;
      case CLIMB:
        currentSuperState = SuperState.CLIMB;
        break;
      case OUTAKE:
        currentSuperState = SuperState.OUTAKE;
        break;
      case SCORE_L1:
        currentSuperState = SuperState.SCORE_L1;
        break;
      case SCORE_L2:
        currentSuperState = SuperState.SCORE_L2;
        break;
      case SCORE_L3:
        currentSuperState = SuperState.SCORE_L3;
        break;
      case SCORE_L4:
        currentSuperState = SuperState.SCORE_L4;
        break;
      case AUTO_SCORE_L1:
        currentSuperState = SuperState.AUTO_SCORE_L1;
        break;
      case AUTO_SCORE_L2:
        currentSuperState = SuperState.AUTO_SCORE_L2;
        break;
      case AUTO_SCORE_L3:
        currentSuperState = SuperState.AUTO_SCORE_L3;
        break;
      case AUTO_SCORE_L4:
        currentSuperState = SuperState.AUTO_SCORE_L4;
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

  // /**
  // * This function handles the CYCLING state of the Superstructure subsystem.
  // * In the CYCLING state, the drive subsystem is set to its default state.
  // *
  // * @return void - This function does not return any value.
  // */
  // public void handleCYCLINGState() {
  // drive.setWantedState(DriveState.DEFAULT);
  // elevator.setWantedState(ElevatorState.IDLE);
  // }

  // /**
  // * This function handles the IDLE state of the Superstructure subsystem.
  // * In the IDLE state, the drive subsystem is set to its IDLE state.
  // *
  // * @return void - This function does not return any value.
  // */
  // public void handleIDLEState() {
  // drive.setWantedState(DriveState.IDLE);
  // elevator.setWantedState(ElevatorState.IDLE);
  // }

  // public void handleElevatorOFFState() {
  // elevator.setWantedState(ElevatorState.OFF);
  // }

  // public void handleElevatorMIDState() {
  // elevator.setWantedState(ElevatorState.MID);
  // }

  // public void handleElevatorUPState() {
  // elevator.setWantedState(ElevatorState.UP);
  // }

  // public void handleElevatorL2State() {
  // elevator.setWantedState(ElevatorState.L2);
  // }

  // public void handeElevatorAlgaeState() {
  // elevator.setWantedState(ElevatorState.ALGAE);
  // }

  // public void handleElevatorL3State() {
  // elevator.setWantedState(ElevatorState.L3);
  // }

  // public void handleOutakingState() {
  // intake.setWantedState(IntakeState.OUTAKE);
  // }

  // public void handleIntakingState() {
  // intake.setWantedState(IntakeState.INTAKE);
  // }

  // public void handleIntakeDefaultState() {
  // intake.setWantedState(IntakeState.DEFAULT);
  // }

  public void handleDefaultState() {
    drive.setWantedState(DriveState.DEFAULT);
    if (pivot.getPivotPosition() < 0.3 && pivot.getPivotPosition() > 0.05) {
      elevator.setWantedState(ElevatorState.DEFAULT);
    } else {
      elevator.setWantedState(ElevatorState.OVER);
    }
    intake.setWantedState(IntakeState.DEFAULT);
    if (elevator.getElevatorPosition() > 10 / 39.37
        || (pivot.getPivotPosition() < 0.3 && pivot.getPivotPosition() > 0.05)) {
      pivot.setWantedState(PivotState.DEFAULT);
    }
    twist.setWantedState(TwistState.UP);
  }

  public void handleAutoL1PlaceState() {
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.AUTO_L1);
    intake.setWantedState(IntakeState.DEFAULT);
    pivot.setWantedFlip(PivotFlip.FRONT);
    pivot.setWantedState(PivotState.L1);
    twist.setWantedState(TwistState.DOWN);
  }

  public void handleAutoL2PlaceState() {
    drive.setWantedState(DriveState.REEF);
    intake.setWantedState(IntakeState.DEFAULT);
    if (Math.hypot(
        drive.getMT2OdometryX() - drive.getReefClosestSetpoint(drive.getMT2Odometry())[0],
        drive.getMT2OdometryY() - drive.getReefClosestSetpoint(drive.getMT2Odometry())[1]) < 1.5
        && drive.getMT2OdometryAngle() - drive.getReefClosestSetpoint(drive.getMT2Odometry())[2] < 1) {
      elevator.setWantedState(ElevatorState.AUTO_L2);
      if (drive.getAutoPlacementSideIsFront()) {
        pivot.setWantedFlip(PivotFlip.FRONT);
      } else {
        pivot.setWantedFlip(PivotFlip.BACK);
      }
      pivot.setWantedState(PivotState.AUTO_L23);
      twist.setWantedState(TwistState.SIDE);
    }
  }

  public void handleAutoL3PlaceState() {
    drive.setWantedState(DriveState.REEF);
    intake.setWantedState(IntakeState.DEFAULT);
    if (Math.hypot(
        drive.getMT2OdometryX() - drive.getReefClosestSetpoint(drive.getMT2Odometry())[0],
        drive.getMT2OdometryY() - drive.getReefClosestSetpoint(drive.getMT2Odometry())[1]) < 1.5
        && drive.getMT2OdometryAngle() - drive.getReefClosestSetpoint(drive.getMT2Odometry())[2] < 1) {
      elevator.setWantedState(ElevatorState.AUTO_L3);
      if (drive.getAutoPlacementSideIsFront()) {
        pivot.setWantedFlip(PivotFlip.FRONT);
      } else {
        pivot.setWantedFlip(PivotFlip.BACK);
      }
      pivot.setWantedState(PivotState.AUTO_L23);
      twist.setWantedState(TwistState.SIDE);
    }
  }

  public void handleAutoL4PlaceState() {
    drive.setWantedState(DriveState.REEF);
    intake.setWantedState(IntakeState.DEFAULT);
    if (Math.hypot(
        drive.getMT2OdometryX() - drive.getReefClosestSetpoint(drive.getMT2Odometry())[0],
        drive.getMT2OdometryY() - drive.getReefClosestSetpoint(drive.getMT2Odometry())[1]) < 1
        && drive.getMT2OdometryAngle() - drive.getReefClosestSetpoint(drive.getMT2Odometry())[2] < 1) {
      elevator.setWantedState(ElevatorState.AUTO_L4);
      if (drive.getAutoPlacementSideIsFront()) {
        pivot.setWantedFlip(PivotFlip.FRONT);
      } else {
        pivot.setWantedFlip(PivotFlip.BACK);
      }
      pivot.setWantedState(PivotState.AUTO_L4);
      twist.setWantedState(TwistState.SIDE);
    }
  }

  public void handleL1PlaceState() {
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.L1);
    intake.setWantedState(IntakeState.DEFAULT);
    pivot.setWantedState(PivotState.L1);
    twist.setWantedState(TwistState.UP);
  }

  public void handleL2PlaceState() {
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.L2);
    intake.setWantedState(IntakeState.DEFAULT);
    pivot.setWantedState(PivotState.L23);
    twist.setWantedState(TwistState.SIDE);
  }

  public void handleL3PlaceState() {
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.L3);
    intake.setWantedState(IntakeState.DEFAULT);
    pivot.setWantedState(PivotState.L23);
    twist.setWantedState(TwistState.SIDE);
  }

  public void handleL4PlaceState() {
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.L4);
    intake.setWantedState(IntakeState.DEFAULT);
    pivot.setWantedState(PivotState.L4);
    twist.setWantedState(TwistState.SIDE);
  }

  public void handleProcessorState() {
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.PROCESSOR);
    intake.setWantedState(IntakeState.DEFAULT);
    pivot.setWantedState(PivotState.PROCESSOR);
    twist.setWantedState(TwistState.SIDE);
  }

  public void handleNetState() {
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.NET);
    intake.setWantedState(IntakeState.DEFAULT);
    pivot.setWantedState(PivotState.NET);
    twist.setWantedState(TwistState.UP);
  }

  public void handleFeederState() { // function for an actual field, comment
    // out the function above when running on
    // an actual field
    drive.setWantedState(DriveState.FEEDER);
    intake.setWantedState(IntakeState.CORAL_INTAKE);
    if (drive.getFieldSide() == "red") { // red side
      if (drive.getMT2OdometryY() > 4.026) { // redside right feeder (field top right)
        if (!(Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) <= 324
            &&
            Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) >= 144)) {
          twist.setWantedState(TwistState.UP);
          pivot.setWantedFlip(PivotFlip.FRONT);
          pivot.setWantedState(PivotState.FEEDER);
          elevator.setWantedState(ElevatorState.DEFAULT);
        } else { // robot back side redside left feeder (fieldside top right)
          if (elevator.getElevatorPosition() <= 13 / 39.37 && pivot.getPivotPosition() > -0.08) {
            elevator.setWantedState(ElevatorState.OVER);
          } else if (elevator.getElevatorPosition() >= 13 / 39.37 &&
              pivot.getPivotPosition() > -0.08) {
            twist.setWantedState(TwistState.DOWN);
            elevator.setWantedState(ElevatorState.OVER);
            pivot.setWantedFlip(PivotFlip.BACK);
            pivot.setWantedState(PivotState.FEEDER);
          } else {
            elevator.setWantedState(ElevatorState.FEEDER_INTAKE);
            twist.setWantedState(TwistState.DOWN);
            pivot.setWantedFlip(PivotFlip.BACK);
            pivot.setWantedState(PivotState.FEEDER);
          }
        }
      } else { // redside left feeder (fieldside bottom right)
        if ((Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) <= 36
            &&
            Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) >= 0)
            ||
            (Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) <= 360
                &&
                Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) >= 216)) {
          twist.setWantedState(TwistState.UP);
          pivot.setWantedFlip(PivotFlip.FRONT);
          pivot.setWantedState(PivotState.FEEDER);
          elevator.setWantedState(ElevatorState.DEFAULT);
        } else { // robot back side redside left (fieldside bottom right)
          if (elevator.getElevatorPosition() <= 13 / 39.37 && pivot.getPivotPosition() > -0.08) {
            elevator.setWantedState(ElevatorState.OVER);
          } else if (elevator.getElevatorPosition() >= 13 / 39.37 &&
              pivot.getPivotPosition() > -0.08) {
            twist.setWantedState(TwistState.DOWN);
            elevator.setWantedState(ElevatorState.OVER);
            pivot.setWantedFlip(PivotFlip.BACK);
            pivot.setWantedState(PivotState.FEEDER);
          } else {
            elevator.setWantedState(ElevatorState.FEEDER_INTAKE);
            twist.setWantedState(TwistState.DOWN);
            pivot.setWantedFlip(PivotFlip.BACK);
            pivot.setWantedState(PivotState.FEEDER);
          }
        }
      }
    } else { // blue side
      if (drive.getMT2OdometryY() < 4.026) { // blue side right feeder (fieldside bottom left)
        if ((Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) <= 324
            &&
            Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) >= 144)) {
          twist.setWantedState(TwistState.UP);
          System.out.println("front");
          pivot.setWantedFlip(PivotFlip.FRONT);
          pivot.setWantedState(PivotState.FEEDER);
          elevator.setWantedState(ElevatorState.DEFAULT);
        } else { // robot back side blueside right (fieldside bottom left)
          if (elevator.getElevatorPosition() <= 13 / 39.37 && pivot.getPivotPosition() > -0.08) {
            elevator.setWantedState(ElevatorState.OVER);
            System.out.println("part 1");
          } else if (elevator.getElevatorPosition() >= 13 / 39.37 &&
              pivot.getPivotPosition() > -0.08) {
            twist.setWantedState(TwistState.DOWN);
            System.out.println("part 2");
            elevator.setWantedState(ElevatorState.OVER);
            pivot.setWantedFlip(PivotFlip.BACK);
            pivot.setWantedState(PivotState.FEEDER);
          } else {
            elevator.setWantedState(ElevatorState.FEEDER_INTAKE);
            System.out.println("part 3");
            twist.setWantedState(TwistState.DOWN);
            pivot.setWantedFlip(PivotFlip.BACK);
            pivot.setWantedState(PivotState.FEEDER);
          }
        }
      } else { // blue side left feeder (fieldside top left)
        if (!((Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) <= 36
            &&
            Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) >= 0)
            ||
            (Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) <= 360
                &&
                Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) >= 216))) {
          twist.setWantedState(TwistState.UP);
          pivot.setWantedFlip(PivotFlip.FRONT);
          pivot.setWantedState(PivotState.FEEDER);
          elevator.setWantedState(ElevatorState.DEFAULT);
        } else { // robot back side blueside left (fieldside top left)
          if (elevator.getElevatorPosition() <= 13 / 39.37 && pivot.getPivotPosition() > -0.08) {
            elevator.setWantedState(ElevatorState.OVER);
          } else if (elevator.getElevatorPosition() >= 13 / 39.37 &&
              pivot.getPivotPosition() > -0.08) {
            twist.setWantedState(TwistState.DOWN);
            elevator.setWantedState(ElevatorState.OVER);
            pivot.setWantedFlip(PivotFlip.BACK);
            pivot.setWantedState(PivotState.FEEDER);
          } else {
            elevator.setWantedState(ElevatorState.FEEDER_INTAKE);
            twist.setWantedState(TwistState.DOWN);
            pivot.setWantedFlip(PivotFlip.BACK);
            pivot.setWantedState(PivotState.FEEDER);
          }
        }
      }
    }
  }

  /*
   */
  public void handleGroundCoralPickupFrontState() {
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.GROUND_INTAKE);
    intake.setWantedState(IntakeState.CORAL_INTAKE);
    pivot.setWantedState(PivotState.GROUND_CORAL_FRONT);
    twist.setWantedState(TwistState.UP);
  }

  public void handleGroundCoralPickupBackState() {
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.GROUND_INTAKE);
    intake.setWantedState(IntakeState.CORAL_INTAKE);

    if (twist.getTwistPosition() < -135 && pivot.getPivotPosition() < -0.1) {
      pivot.setWantedState(PivotState.GROUND_CORAL_BACK);
    } else if (twist.getTwistPosition() > -45 || pivot.getPivotPosition() < -0.1) {
      pivot.setWantedState(PivotState.GROUND_CORAL_PREP_BACK);
    }

    if (pivot.getPivotPosition() < 0.1) {
      twist.setWantedState(TwistState.DOWN);
    }
  }

  public void handleGroundAlgaePickupState() {
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.GROUND_INTAKE);
    intake.setWantedState(IntakeState.ALGAE_INTAKE);
    pivot.setWantedState(PivotState.GROUND_ALGAE);
    twist.setWantedState(TwistState.UP);
  }

  public void handleL2AlgaePickupState() {
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.L2_ALGAE);
    intake.setWantedState(IntakeState.ALGAE_INTAKE);
    pivot.setWantedState(PivotState.REEF_ALGAE);
    twist.setWantedState(TwistState.UP);
  }

  public void handleL3AlgaePickupState() {
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.L3_ALGAE);
    intake.setWantedState(IntakeState.ALGAE_INTAKE);
    pivot.setWantedState(PivotState.REEF_ALGAE);
    twist.setWantedState(TwistState.UP);
  }

  public void handleDeployClimberState() {

  }

  public void handleClimbState() {

  }

  public void handleOutakeState() {
    intake.setWantedState(IntakeState.OUTAKE);
  }

  public void handleAutoL1ScoreState() {
    drive.setWantedState(DriveState.DEFAULT);
    intake.setWantedState(IntakeState.OUTAKE);
  }

  public void handleAutoL2ScoreState() {
    drive.setWantedState(DriveState.DEFAULT);
    pivot.setWantedState(PivotState.AUTO_SCORE_L23);
  }

  public void handleAutoL3ScoreState() {
    drive.setWantedState(DriveState.DEFAULT);
    pivot.setWantedState(PivotState.AUTO_SCORE_L23);
  }

  public void handleAutoL4ScoreState() {
    drive.setWantedState(DriveState.DEFAULT);
    pivot.setWantedState(PivotState.AUTO_SCORE_L4);
  }

  public void handleScoreL1State() {
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.SCORE_L1);
    intake.setWantedState(IntakeState.OUTAKE);
    pivot.setWantedState(PivotState.SCORE_L1);
  }

  public void handleScoreL2State() {
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.SCORE_L2);
    pivot.setWantedState(PivotState.SCORE_L23);
  }

  public void handleScoreL3State() {
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.SCORE_L3);
    pivot.setWantedState(PivotState.SCORE_L23);
  }

  public void handleScoreL4State() {
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.SCORE_L4);
    pivot.setWantedState(PivotState.SCORE_L4);
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
