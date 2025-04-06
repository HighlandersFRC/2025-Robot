package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

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

  public enum SuperState {
    DEFAULT,
    AUTO_L1_PLACE,
    AUTO_L1_PLACE_MORE,
    AUTO_L2_PLACE,
    AUTO_L3_PLACE,
    AUTO_L4_PLACE,
    L1_PLACE,
    L2_PLACE,
    L3_PLACE,
    L4_PLACE,
    PROCESSOR,
    AUTO_PROCESSOR,
    AUTO_PROCESSOR_MORE,
    OUTAKE_DRIVE,
    NET,
    AUTO_NET,
    AUTO_NET_MORE,
    FEEDER_ALIGN,
    FEEDER_AUTO, // TODO: do the side to side motion
    FEEDER,
    GROUND_CORAL_PICKUP_FRONT,
    GROUND_CORAL_PICKUP_BACK,
    AUTO_GROUND_CORAL_PICKUP_FRONT,
    AUTO_GROUND_CORAL_PICKUP_BACK,
    GROUND_ALGAE_PICKUP_FRONT,
    GROUND_ALGAE_PICKUP_BACK,
    L2_ALGAE_PICKUP,
    L3_ALGAE_PICKUP,
    AUTO_ALGAE_PICKUP,
    AUTO_ALGAE_PICKUP_MORE,
    AUTO_ALGAE_PICKUP_MORE_MORE,
    DEPLOY_CLIMBER,
    CLIMB,
    CLIMBER_IDLE,
    OUTAKE,
    SCORE_L1,
    SCORE_L2,
    SCORE_L3,
    SCORE_L4,
    AUTO_SCORE_L1,
    AUTO_SCORE_L2,
    AUTO_SCORE_L3,
    AUTO_SCORE_MORE_L3,
    AUTO_SCORE_L4,
    IDLE,
    OUTAKE_IDLE,
    MANUAL_PLACE,
    MANUAL_RESET,
    AUTO_FEEDER,
    RUN_CLIMB_BACK,
    AUTO_CLIMB,
    DEFAULT_DRIVE,
    LOLLIOP_PICKUP,
    ZERO
  }

  private SuperState wantedSuperState = SuperState.DEFAULT;
  private SuperState currentSuperState = SuperState.DEFAULT;

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

  public boolean hasCoral() {
    return manipulator.hasCoral();
  }

  private void applyStates() {
    switch (currentSuperState) {
      case DEFAULT:
        handleDefaultState();
        break;
      case ZERO:
        handleZeroState();
        break;
      case DEFAULT_DRIVE:
        handleDefaultDriveState();
        break;
      case AUTO_L1_PLACE:
        handleAutoL1PlaceState();
        break;
      case AUTO_L1_PLACE_MORE:
        handleAutoL1PlaceMoreState();
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
      case OUTAKE_DRIVE:
        handleOutakeDriveState();
        break;
      case NET:
        handleNetState();
        break;
      case AUTO_PROCESSOR:
        handleAutoProcessorState();
        break;
      case AUTO_PROCESSOR_MORE:
        handleAutoProcessorMoreState();
        break;
      case AUTO_NET:
        handleAutoNetState();
        break;
      case AUTO_NET_MORE:
        handleAutoNetStateMore();
        break;
      case FEEDER_ALIGN:
        handleFeederAlignState();
        break;
      case FEEDER_AUTO:
        handleFeederAutoState();
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
      case AUTO_GROUND_CORAL_PICKUP_FRONT:
        handleAutoGroundCoralPickupFrontState();
        break;
      case AUTO_GROUND_CORAL_PICKUP_BACK:
        handleAutoGroundCoralPickupBackState();
        break;
      case GROUND_ALGAE_PICKUP_FRONT:
        handleGroundAlgaePickupFrontState();
        break;
      case GROUND_ALGAE_PICKUP_BACK:
        handleGroundAlgaePickupBackState();
        break;
      case L2_ALGAE_PICKUP:
        handleL2AlgaePickupState();
        break;
      case L3_ALGAE_PICKUP:
        handleL3AlgaePickupState();
        break;
      case AUTO_ALGAE_PICKUP:
        handleAutoAlgaePickupState();
        break;
      case AUTO_ALGAE_PICKUP_MORE:
        handleAutoAlgaePickupMoreState();
        break;
      case AUTO_ALGAE_PICKUP_MORE_MORE:
        handleAutoAlgaePickupMoreMoreState();
        break;
      case DEPLOY_CLIMBER:
        handleDeployClimberState();
        break;
      case CLIMB:
        handleClimbState();
        break;
      case CLIMBER_IDLE:
        handleClimberIdleState();
        break;
      case AUTO_CLIMB:
        handleAutoClimbState();
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
      case AUTO_SCORE_MORE_L3:
        handleAutoL3ScoreMoreState();
        break;
      case AUTO_SCORE_L4:
        handleAutoL4ScoreState();
        break;
      case IDLE:
        handleIdleState();
        break;
      case OUTAKE_IDLE:
        handleOutakeIdleState();
        break;
      case MANUAL_PLACE:
        handleManualPlaceState();
        break;
      case MANUAL_RESET:
        handleManualResetState();
        break;
      case AUTO_FEEDER:
        handleAutoFeederState();
        break;
      case RUN_CLIMB_BACK:
        handleRunClimbBack();
        break;
      case LOLLIOP_PICKUP:
        handleLollipopPickup();
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
      case ZERO:
        if (intake.getZeroed() && elevator.getZeroed() && firstTimeZero) {
          firstTimeZero = false;
          zeroTime = Timer.getFPGATimestamp();
        }
        if (Timer.getFPGATimestamp() - zeroTime > 1.0 && intake.getZeroed() && elevator.getZeroed()) {
          currentSuperState = SuperState.DEFAULT;
          wantedSuperState = SuperState.DEFAULT;
        } else {
          currentSuperState = SuperState.ZERO;
        }
        break;
      case DEFAULT_DRIVE:
        currentSuperState = SuperState.DEFAULT_DRIVE;
        break;
      case AUTO_L1_PLACE:
        if (Math.abs(Math.abs(pivot.getPivotPosition()) - Constants.SetPoints.PivotPosition.kL1.rotations) < 10.0
            / 360.0) {
          currentSuperState = SuperState.AUTO_L1_PLACE_MORE;
          wantedSuperState = SuperState.AUTO_L1_PLACE_MORE;
        } else {
          currentSuperState = SuperState.AUTO_L1_PLACE;
        }
        break;
      case AUTO_L1_PLACE_MORE:
        if (drive.hitSetPointSemiGenerous(drive.getL1ReefClosestSetpointMore(drive.getMT2Odometry(), false)[0],
            drive.getL1ReefClosestSetpointMore(drive.getMT2Odometry(), false)[1],
            drive.getL1ReefClosestSetpointMore(drive.getMT2Odometry(), false)[2])) {
          currentSuperState = SuperState.AUTO_SCORE_L1;
          wantedSuperState = SuperState.AUTO_SCORE_L1;
        } else {
          currentSuperState = SuperState.AUTO_L1_PLACE_MORE;
        }
        break;
      case OUTAKE_DRIVE:
        currentSuperState = SuperState.OUTAKE_DRIVE;
        break;
      case AUTO_L2_PLACE:
        if (drive.hitSetPoint(drive.getReefClosestSetpoint(drive.getMT2Odometry(), OI
            .getDriverA())[0],
            drive.getReefClosestSetpoint(drive.getMT2Odometry(), OI
                .getDriverA())[1],
            drive.getReefClosestSetpoint(drive.getMT2Odometry(), OI
                .getDriverA())[2])
            && elevator.getElevatorPosition() > Constants.SetPoints.ElevatorPosition.kAUTOL2.meters - 3.0 / 39.37) {
          currentSuperState = SuperState.AUTO_SCORE_L2;
          wantedSuperState = SuperState.AUTO_SCORE_L2;
        } else {
          currentSuperState = SuperState.AUTO_L2_PLACE;
        }
        break;
      case AUTO_L3_PLACE:
        // System.out.println(
        // "Hit Set Point: " +
        // drive.hitSetPoint(drive.getReefL3ClosestSetpoint(drive.getMT2Odometry())[0],
        // drive.getReefL3ClosestSetpoint(drive.getMT2Odometry())[1],
        // drive.getReefL3ClosestSetpoint(
        // drive.getMT2Odometry())[2])
        // + " Elevator Position: " + elevator.getElevatorPosition() * 39.37);
        // if
        // (drive.hitSetPoint(drive.getReefL3ClosestSetpoint(drive.getMT2Odometry())[0],
        // drive.getReefL3ClosestSetpoint(drive.getMT2Odometry())[1],
        // drive.getReefL3ClosestSetpoint(drive
        // .getMT2Odometry())[2])
        // && elevator.getElevatorPosition() > ElevatorPosition.kAUTOL3.meters - 2 /
        // 39.37) {
        // currentSuperState = SuperState.AUTO_SCORE_L3;
        // wantedSuperState = SuperState.AUTO_SCORE_L3;
        // // hitAutoSetpointTime = Timer.getFPGATimestamp();
        // } else {
        // currentSuperState = SuperState.AUTO_L3_PLACE;
        // }
        // break;
        if (drive.hitSetPoint(drive.getReefL3ClosestSetpoint(drive.getMT2Odometry(), OI
            .getDriverA())[0],
            drive.getReefL3ClosestSetpoint(drive.getMT2Odometry(), OI
                .getDriverA())[1],
            drive.getReefL3ClosestSetpoint(drive.getMT2Odometry(), OI
                .getDriverA())[2])
            && elevator.getElevatorPosition() > Constants.SetPoints.ElevatorPosition.kAUTOL3.meters - 3.0 / 39.37) {
          currentSuperState = SuperState.AUTO_SCORE_L3;
          wantedSuperState = SuperState.AUTO_SCORE_L3;
        } else {
          currentSuperState = SuperState.AUTO_L3_PLACE;
        }
        break;
      case AUTO_L4_PLACE:
        if (drive.hitSetPoint(drive.getReefL4ClosestSetpoint(drive.getMT2Odometry(), OI
            .getDriverA())[0],
            drive.getReefL4ClosestSetpoint(drive.getMT2Odometry(), OI
                .getDriverA())[1],
            drive.getReefL4ClosestSetpoint(drive.getMT2Odometry(), OI
                .getDriverA())[2])
            && elevator.getElevatorPosition() > Constants.SetPoints.ElevatorPosition.kAUTOL4.meters - 3.0 / 39.37
            && Math.abs(peripherals.getPigeonPitch()) < 2.0) {
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
      case AUTO_PROCESSOR_MORE:
        currentSuperState = SuperState.AUTO_PROCESSOR_MORE;
        break;
      case NET:
        currentSuperState = SuperState.NET;
        break;
      case AUTO_PROCESSOR:
        if (OI.isBlueSide()) {
          if (drive.hitSetPointGenerous(Constants.Reef.processorBlueFrontPlacingPosition.getX(),
              Constants.Reef.processorBlueFrontPlacingPosition
                  .getY(),
              drive.getMT2OdometryAngle())
              && elevator.getElevatorPosition() > Constants.SetPoints.ElevatorPosition.kPROCESSOR.meters
                  - 5.0 / 39.37
              && (drive.getAngleDifferenceDegrees(Math.toDegrees(drive.getMT2OdometryAngle()),
                  Constants.Reef.processorBlueFrontPlacingPosition.getRotation().getDegrees()) < 10.0
                  || drive
                      .getAngleDifferenceDegrees(Math.toDegrees(drive.getMT2OdometryAngle()),
                          Constants.Reef.processorBlueBackPlacingPosition.getRotation().getDegrees()) < 10.0)) {
            wantedSuperState = SuperState.AUTO_PROCESSOR_MORE;
            currentSuperState = SuperState.AUTO_PROCESSOR_MORE;
          } else {
            currentSuperState = SuperState.AUTO_PROCESSOR;
          }
        } else {
          if (drive.hitSetPoint(Constants.Reef.processorRedFrontPlacingPosition.getX(),
              Constants.Reef.processorRedFrontPlacingPosition
                  .getY(),
              drive.getMT2OdometryAngle())
              && elevator.getElevatorPosition() > Constants.SetPoints.ElevatorPosition.kPROCESSOR.meters
                  - 5.0 / 39.37
              && (drive.getAngleDifferenceDegrees(Math.toDegrees(drive.getMT2OdometryAngle()),
                  Constants.Reef.processorRedFrontPlacingPosition.getRotation().getDegrees()) < 10.0
                  || drive
                      .getAngleDifferenceDegrees(Math.toDegrees(drive.getMT2OdometryAngle()),
                          Constants.Reef.processorRedBackPlacingPosition.getRotation().getDegrees()) < 10.0)) {
            wantedSuperState = SuperState.AUTO_PROCESSOR_MORE;
            currentSuperState = SuperState.AUTO_PROCESSOR_MORE;
          } else {
            currentSuperState = SuperState.AUTO_PROCESSOR;
          }
        }
        break;
      case AUTO_NET:
        if (drive.hitSetPointUltraGenerous(drive.getNetXSetpoint(), drive.getMT2OdometryY(),
            drive.getNetThetaSetpoint())
            && elevator.getElevatorPosition() > Constants.SetPoints.ElevatorPosition.kNET.meters - 10.0 / 39.37) {
          wantedSuperState = SuperState.AUTO_NET_MORE;
          currentSuperState = SuperState.AUTO_NET_MORE;
        } else {
          currentSuperState = SuperState.AUTO_NET;
        }
        break;
      case AUTO_NET_MORE:
        currentSuperState = SuperState.AUTO_NET_MORE;
        break;
      case FEEDER_ALIGN:
        currentSuperState = SuperState.FEEDER_ALIGN;
        break;
      case FEEDER_AUTO:
        currentSuperState = SuperState.FEEDER_AUTO;
        break;
      case FEEDER:
        currentSuperState = SuperState.FEEDER;
        break;
      case GROUND_CORAL_PICKUP_FRONT:
        if (intake.hasCoral() || manipulator.getArmItem() == ArmItem.CORAL || continueFeeding) {
          currentSuperState = SuperState.DEFAULT;
        } else {
          currentSuperState = SuperState.GROUND_CORAL_PICKUP_FRONT;
        }
        break;
      case GROUND_CORAL_PICKUP_BACK:
        currentSuperState = SuperState.GROUND_CORAL_PICKUP_BACK;
        break;
      case AUTO_GROUND_CORAL_PICKUP_FRONT:
        if (intake.hasCoral() || manipulator.hasCoral() || continueFeeding) {
          currentSuperState = SuperState.DEFAULT;
        } else if (peripherals.getFrontGamePieceCamResult().hasTargets()) {
          currentSuperState = SuperState.AUTO_GROUND_CORAL_PICKUP_FRONT;
        } else {
          currentSuperState = SuperState.GROUND_CORAL_PICKUP_FRONT;
        }
        break;
      case AUTO_GROUND_CORAL_PICKUP_BACK:
        currentSuperState = SuperState.AUTO_GROUND_CORAL_PICKUP_BACK;
        break;
      case GROUND_ALGAE_PICKUP_FRONT:
        currentSuperState = SuperState.GROUND_ALGAE_PICKUP_FRONT;
        break;
      case GROUND_ALGAE_PICKUP_BACK:
        currentSuperState = SuperState.GROUND_ALGAE_PICKUP_BACK;
        break;
      case L2_ALGAE_PICKUP:
        currentSuperState = SuperState.L2_ALGAE_PICKUP;
        break;
      case L3_ALGAE_PICKUP:
        currentSuperState = SuperState.L3_ALGAE_PICKUP;
        break;
      case AUTO_ALGAE_PICKUP:
        if (drive.hitSetPointGenerous(drive.getAlgaeClosestSetpoint(drive.getMT2Odometry())[0],
            drive.getAlgaeClosestSetpoint(drive.getMT2Odometry())[1],
            drive.getAlgaeClosestSetpoint(drive.getMT2Odometry())[2])
            && elevator.getElevatorPosition() > Constants.SetPoints.ElevatorPosition.kL2ALGAE.meters - 5.0 / 39.37
            && Math.abs(pivot.getPivotPosition())
                - Math.abs(Constants.SetPoints.PivotPosition.kREEFALGAE.rotations) < 0.06) {
          currentSuperState = SuperState.AUTO_ALGAE_PICKUP_MORE;
          wantedSuperState = SuperState.AUTO_ALGAE_PICKUP_MORE;
        } else {
          currentSuperState = SuperState.AUTO_ALGAE_PICKUP;
        }
        break;
      case AUTO_ALGAE_PICKUP_MORE:
        currentSuperState = SuperState.AUTO_ALGAE_PICKUP_MORE;
        break;
      case AUTO_ALGAE_PICKUP_MORE_MORE:
        currentSuperState = SuperState.AUTO_ALGAE_PICKUP_MORE_MORE;
        break;
      case DEPLOY_CLIMBER:
        if (climber.getPosition() > -400) {
          currentSuperState = SuperState.DEPLOY_CLIMBER;
        } else {
          wantedSuperState = SuperState.CLIMBER_IDLE;
          currentSuperState = SuperState.CLIMBER_IDLE;
        }
        break;
      case CLIMB:
        if (climber.getPosition() < -105) {
          currentSuperState = SuperState.CLIMB;
        } else {
          wantedSuperState = SuperState.CLIMBER_IDLE;
          currentSuperState = SuperState.CLIMBER_IDLE;
        }
        break;
      case AUTO_CLIMB:
        currentSuperState = SuperState.AUTO_CLIMB;
        break;
      case CLIMBER_IDLE:
        currentSuperState = SuperState.CLIMBER_IDLE;
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
        // if (drive.getDistanceFromL23Setpoint() < 7 / 39.37) {
        currentSuperState = SuperState.AUTO_SCORE_L3;
        // } else {
        // currentSuperState = SuperState.AUTO_SCORE_MORE_L3;
        // wantedSuperState = SuperState.AUTO_SCORE_MORE_L3;
        // }
        break;
      case AUTO_SCORE_MORE_L3:
        currentSuperState = SuperState.AUTO_SCORE_MORE_L3;
        break;
      case AUTO_SCORE_L4:
        currentSuperState = SuperState.AUTO_SCORE_L4;
        break;
      case IDLE:
        currentSuperState = SuperState.IDLE;
        break;
      case OUTAKE_IDLE:
        currentSuperState = SuperState.OUTAKE_IDLE;
        break;
      case MANUAL_PLACE:
        currentSuperState = SuperState.MANUAL_PLACE;
        break;
      case MANUAL_RESET:
        currentSuperState = SuperState.MANUAL_RESET;
        break;
      case AUTO_FEEDER:
        currentSuperState = SuperState.AUTO_FEEDER;
        break;
      case RUN_CLIMB_BACK:
        currentSuperState = SuperState.RUN_CLIMB_BACK;
        break;
      case LOLLIOP_PICKUP:
        currentSuperState = SuperState.LOLLIOP_PICKUP;
        break;
      default:
        currentSuperState = SuperState.IDLE;
        break;
    }
    return currentSuperState;

  }

  public boolean placedCoralL4() {
    // return
    // drive.hitSetPoint(drive.getReefL4ClosestSetpoint(drive.getMT2Odometry())[0],
    // drive.getReefL4ClosestSetpoint(drive.getMT2Odometry())[1],
    // drive.getReefL4ClosestSetpoint(drive.getMT2Odometry())[2]) &&
    // elevator.getElevatorPosition() > 53 / 39.37
    // &&

    // Pivot has abs to account for placing backwards
    return Math
        .abs(Math.abs(pivot.getPivotPosition())
            - Constants.SetPoints.PivotPosition.kAUTOL4SCORE.rotations) < (20.0 / 360.0);
  }

  public boolean placedCoralL2() {
    // return
    // drive.hitSetPoint(drive.getReefL4ClosestSetpoint(drive.getMT2Odometry())[0],
    // drive.getReefL4ClosestSetpoint(drive.getMT2Odometry())[1],
    // drive.getReefL4ClosestSetpoint(drive.getMT2Odometry())[2]) &&
    // elevator.getElevatorPosition() > 53 / 39.37
    // &&

    // Pivot has abs to account for placing backwards
    return Math
        .abs(Math.abs(pivot.getPivotPosition())
            - Constants.SetPoints.PivotPosition.kAUTOL2SCORE.rotations) < (5.0 / 360.0);
  }

  public boolean isClimbing = false;

  public void handleDefaultDriveState() {
    drive.setWantedState(DriveState.DEFAULT);
  }

  private double zeroTime = Timer.getFPGATimestamp();
  private boolean firstTimeZero = true;

  public void handleZeroState() {
    elevator.setWantedState(ElevatorState.ZERO);
    intake.setWantedState(IntakeState.ZERO);
    pivot.setWantedState(PivotState.DEFAULT);
    twist.setWantedState(TwistState.SIDE);
  }

  public void handleDefaultState() {
    // peripherals.setBackCamPipline(0);
    lights.setWantedState(LightsState.DEFAULT);
    drive.setWantedState(DriveState.DEFAULT);
    // pivot.setWantedFlip(PivotFlip.FRONT);
    // if ()
    if (algaeMode) {
      if (/* Math.abs(twist.getTwistPosition()) < 45 && */ Math.abs(pivot.getPivotPosition()) > 90.0 / 360.0) {
        elevator.setWantedState(ElevatorState.PREHANDOFF);
      } else {
        elevator.setWantedState(ElevatorState.GROUND_CORAL_INTAKE);
      }
      manipulator.setWantedState(ManipulatorState.DEFAULT);
      intake.setWantedState(IntakeState.DEFAULT);
      // if (isClimbing) {
      // pivot.setWantedState(PivotState.DEFAULT_CLIMB);
      // } else {
      if (Math.abs(twist.getTwistPosition()) < 30.0) {
        pivot.setWantedState(PivotState.DEFAULT);
        firstTimeDefault = false;
      } else if (firstTimeDefault) {
        pivot.setWantedState(PivotState.PREP);
      }
      // }

      if (Math.abs(pivot.getPivotPosition()) > 40.0 / 360.0) {
      }
      // if (Math.abs(pivot.getPivotPosition()) < 90.0 / 360.0) {
      twist.setWantedState(TwistState.SIDE);
    } else if (manipulator.hasCoralSticky()) {
      // if (/* Math.abs(twist.getTwistPosition()) < 45 && */
      // Math.abs(pivot.getPivotPosition()) < 90.0 / 360.0) {
      if (/* Math.abs(twist.getTwistPosition()) < 45 && */ Math.abs(pivot.getPivotPosition()) > 90.0 / 360.0) {
        elevator.setWantedState(ElevatorState.PREHANDOFF);
      } else {
        elevator.setWantedState(ElevatorState.L3);
      }
      // } else {
      // elevator.setWantedState(ElevatorState.GROUND_CORAL_INTAKE);
      // }
      manipulator.setWantedState(ManipulatorState.CORAL_INTAKE);

      // if (isClimbing) {
      // pivot.setWantedState(PivotState.DEFAULT_CLIMB);
      // } else {
      // if (Math.abs(twist.getTwistPosition()) < 30.0) {
      pivot.setWantedState(PivotState.DEFAULT);
      //   firstTimeDefault = false;
      // } else if (firstTimeDefault) {
      //   pivot.setWantedState(PivotState.PREP);
      // }
      intake.setWantedState(IntakeState.DEFAULT);
      // }

      // if (Math.abs(pivot.getPivotPosition()) > 40.0 / 360.0) {
      // }
      // if (Math.abs(pivot.getPivotPosition()) < 90.0 / 360.0) {
      twist.setWantedState(TwistState.SIDE);
    } else {
      // Logger.recordOutput("Can Handoff", Math.abs(twist.getTwistPosition() + 90) < 10
      //     && Math.abs(pivot.getPivotPosition() - Constants.SetPoints.PivotPosition.kHANDOFF.rotations) < 0.02
      //     && Math.abs(elevator.getElevatorPosition() - Constants.SetPoints.ElevatorPosition.kPREHANDOFF.meters) < 0.05);
      // Logger.recordOutput("Twist Correct", Math.abs(twist.getTwistPosition() + 90) < 10);
      // Logger.recordOutput("Pivot Correct",
      //     Math.abs(pivot.getPivotPosition() - Constants.SetPoints.PivotPosition.kHANDOFF.rotations) < 0.02);
      // Logger.recordOutput("Elevator Correct",
      //     Math.abs(elevator.getElevatorPosition() - Constants.SetPoints.ElevatorPosition.kPREHANDOFF.meters) < 0.05);
      twist.setWantedState(TwistState.UP);
      manipulator.setWantedState(ManipulatorState.DEFAULT);
      // Wait for the elevator to come up to move the pivot
      if (Math.abs(elevator.getElevatorPosition()) > 15.0 / 39.37) {
        pivot.setWantedState(PivotState.HANDOFF);
      }
      // If everything is in pre-handoff position and there is a coral to be picked
      // up, move the elevator down to pick up.
      if (Math.abs(twist.getTwistPosition() + 90) < 10
          && Math.abs(pivot.getPivotPosition() - Constants.SetPoints.PivotPosition.kHANDOFF.rotations) < 0.05
          && Math.abs(intake.getPosition() - Constants.SetPoints.IntakeSetpoints.INTAKE_UP) < 0.05
          && (intake.hasCoral() || continueFeeding)) {
        elevator.setWantedState(ElevatorState.HANDOFF);
      } else {
        elevator.setWantedState(ElevatorState.PREHANDOFF);
      }
      // Once the elevator makes it down to the handoff position, outake into the arm
      if ((Math.abs(twist.getTwistPosition() + 90) < 10
          && Math.abs(pivot.getPivotPosition() - Constants.SetPoints.PivotPosition.kHANDOFF.rotations) < 0.05
          && Math.abs(elevator.getElevatorPosition() - Constants.SetPoints.ElevatorPosition.kHANDOFF.meters) < 2.0
              / 39.37
          && Math.abs(intake.getPosition() - Constants.SetPoints.IntakeSetpoints.INTAKE_UP) < 0.05)
          || continueFeeding) {
        intake.setWantedState(IntakeState.HANDOFF);
        continueFeeding = true;
      } else {
        intake.setWantedState(IntakeState.DEFAULT);
      }
      // Timeout for the pass off
      if (continueFeeding && handoffInitTime == 0.0) {
        handoffInitTime = Timer.getFPGATimestamp();
      } else {
        if (Timer.getFPGATimestamp() - handoffInitTime > 0.5) {
          continueFeeding = false;
          handoffInitTime = 0.0;
        }
      }
    }
    // }
    climber.setWantedState(ClimbState.IDLE);
  }

  public void handleAutoL1PlaceState() {
    manipulator.inL1State = true;
    lights.setWantedState(LightsState.PLACING);
    if (Math.hypot(OI.getDriverLeftX(), OI.getDriverLeftY()) > 0.05 || Math.hypot(OI.getDriverLeftX(),
        OI.getDriverLeftY()) > 0.05) {
      drive.setWantedState(DriveState.DEFAULT);
    } else {
      drive.setWantedState(DriveState.AUTO_L1);
    }
    elevator.setWantedState(ElevatorState.AUTO_L1);
    manipulator.setWantedState(ManipulatorState.DEFAULT);
    // System.out.println(drive.getAngleDifferenceDegrees(Math.toDegrees(drive.getMT2OdometryAngle()),
    // Math.toDegrees(drive.getReefClosestSetpointFrontOnly(drive.getMT2Odometry())[2])));
    if (drive.getAngleDifferenceDegrees(Math.toDegrees(drive.getMT2OdometryAngle()),
        Math.toDegrees(drive.getReefClosestSetpointFrontOnly(drive.getMT2Odometry())[2])) < 90) {
      pivot.setWantedFlip(PivotFlip.FRONT);
      if (Math.abs(pivot.getPivotPosition()) > 30.0 / 360.0) {
        twist.setWantedState(TwistState.UP);
      }
    } else {
      pivot.setWantedFlip(PivotFlip.BACK);
      if (Math.abs(pivot.getPivotPosition()) > 30.0 / 360.0) {
        twist.setWantedState(TwistState.DOWN);
      }
    }
    if (elevator.getElevatorPosition() > Constants.SetPoints.ElevatorPosition.kL1.meters - 10 / 39.37) {
      pivot.setWantedState(PivotState.AUTO_L1);
    }
  }

  public void handleAutoL1PlaceMoreState() {
    manipulator.inL1State = true;
    lights.setWantedState(LightsState.PLACING);
    if (Math.hypot(OI.getDriverLeftX(), OI.getDriverLeftY()) > 0.05 || Math.hypot(OI.getDriverLeftX(),
        OI.getDriverLeftY()) > 0.05) {
      drive.setWantedState(DriveState.DEFAULT);
    } else {
      drive.setWantedState(DriveState.AUTO_L1_MORE);
    }
    elevator.setWantedState(ElevatorState.AUTO_L1);
    manipulator.setWantedState(ManipulatorState.DEFAULT);
    // System.out.println(drive.getAngleDifferenceDegrees(Math.toDegrees(drive.getMT2OdometryAngle()),
    // Math.toDegrees(drive.getReefClosestSetpointFrontOnly(drive.getMT2Odometry())[2])));
    if (drive.getAngleDifferenceDegrees(Math.toDegrees(drive.getMT2OdometryAngle()),
        Math.toDegrees(drive.getReefClosestSetpointFrontOnly(drive.getMT2Odometry())[2])) < 90) {
      pivot.setWantedFlip(PivotFlip.FRONT);
      if (Math.abs(pivot.getPivotPosition()) > 30.0 / 360.0) {
        twist.setWantedState(TwistState.UP);
      }
    } else {
      pivot.setWantedFlip(PivotFlip.BACK);
      if (Math.abs(pivot.getPivotPosition()) > 30.0 / 360.0) {
        twist.setWantedState(TwistState.DOWN);
      }
    }
    // if (elevator.getElevatorPosition() >
    // Constants.SetPoints.ElevatorPosition.kL1.meters - 10 / 39.37) {
    pivot.setWantedState(PivotState.AUTO_L1);
    // }
  }

  public void handleAutoL2PlaceState() {
    lights.setWantedState(LightsState.PLACING);
    if (Math.hypot(OI.getDriverLeftX(), OI.getDriverLeftY()) > 0.05 || Math.hypot(OI.getDriverLeftX(),
        OI.getDriverLeftY()) > 0.05) {
      drive.setWantedState(DriveState.DEFAULT);
    } else {
      drive.setWantedState(DriveState.REEF);
    }
    if (Math.abs(pivot.getPivotPosition()) > 80.0 / 360.0
        && Math.abs(elevator.getElevatorPosition() - Constants.SetPoints.ELEVATOR_AUTO_L2_POSITION_M) < 0.1) {
      manipulator.setWantedState(ManipulatorState.OUTAKE);
    } else {
      manipulator.setWantedState(ManipulatorState.DEFAULT);
    }
    if (Math.hypot(
        drive.getMT2OdometryX() - drive.getReefClosestSetpoint(drive.getMT2Odometry(), false)[0],
        drive.getMT2OdometryY() - drive.getReefClosestSetpoint(drive.getMT2Odometry(), false)[1]) < 1.5
        && drive.getAngleDifferenceDegrees(Math.toDegrees(drive.getMT2OdometryAngle()),
            Math.toDegrees(drive.getReefClosestSetpoint(drive.getMT2Odometry(), false)[2])) < 50.0) {
      elevator.setWantedState(ElevatorState.AUTO_L2);
      if (drive.getAutoPlacementSideIsFront()) {
        pivot.setWantedFlip(PivotFlip.FRONT);
      } else {
        pivot.setWantedFlip(PivotFlip.BACK);
      }
      if (elevator.getElevatorPosition() > Constants.SetPoints.ElevatorPosition.kAUTOL2.meters - 10 / 39.37) {
        pivot.setWantedState(PivotState.AUTO_L2);
      }
      twist.setWantedState(TwistState.SIDE);
    }
  }

  public void handleAutoL3PlaceState() {
    lights.setWantedState(LightsState.PLACING);
    if (Math.hypot(OI.getDriverLeftX(), OI.getDriverLeftY()) > 0.05 || Math.hypot(OI.getDriverLeftX(),
        OI.getDriverLeftY()) > 0.05) {
      drive.setWantedState(DriveState.DEFAULT);
    } else {
      drive.setWantedState(DriveState.L3_REEF);
    }
    if (Math.abs(pivot.getPivotPosition()) > 80.0 / 360.0) {
      manipulator.setWantedState(ManipulatorState.OUTAKE);
    } else {
      manipulator.setWantedState(ManipulatorState.DEFAULT);
    }
    if (Math.hypot(
        drive.getMT2OdometryX() - drive.getReefClosestSetpoint(drive.getMT2Odometry(), false)[0],
        drive.getMT2OdometryY() - drive.getReefClosestSetpoint(drive.getMT2Odometry(), false)[1]) < 1.5
        && drive.getAngleDifferenceDegrees(Math.toDegrees(drive.getMT2OdometryAngle()),
            Math.toDegrees(drive.getReefClosestSetpoint(drive.getMT2Odometry(), false)[2])) < 50.0) {
      elevator.setWantedState(ElevatorState.AUTO_L3);
      if (drive.getAutoPlacementSideIsFront()) {
        pivot.setWantedFlip(PivotFlip.FRONT);
      } else {
        pivot.setWantedFlip(PivotFlip.BACK);
      }
      if (elevator.getElevatorPosition() > Constants.SetPoints.ElevatorPosition.kAUTOL3.meters - 10 / 39.37) {
        pivot.setWantedState(PivotState.AUTO_L3);
      }
      twist.setWantedState(TwistState.SIDE);
    }
  }

  public void handleAutoL4PlaceState() {
    if (OI.driverLB.getAsBoolean()) {
      setWantedState(SuperState.AUTO_SCORE_L4);
    }
    lights.setWantedState(LightsState.PLACING);
    if (Math.hypot(OI.getDriverLeftX(), OI.getDriverLeftY()) > 0.05 || Math.hypot(OI.getDriverLeftX(),
        OI.getDriverLeftY()) > 0.05) {
      drive.setWantedState(DriveState.DEFAULT);
    } else {
      drive.setWantedState(DriveState.L4_REEF);
    }
    manipulator.setWantedState(ManipulatorState.DEFAULT);
    if (Math.hypot(
        drive.getMT2OdometryX() - drive.getReefL4ClosestSetpoint(drive.getMT2Odometry(), false)[0],
        drive.getMT2OdometryY() - drive.getReefL4ClosestSetpoint(drive.getMT2Odometry(), false)[1]) < 0.8
        && drive.getAngleDifferenceDegrees(Math.toDegrees(drive.getMT2OdometryAngle()),
            Math.toDegrees(drive.getReefL4ClosestSetpoint(drive.getMT2Odometry(), false)[2])) < 50.0) {
      elevator.setWantedState(ElevatorState.AUTO_L4);
      if (drive.getAutoPlacementSideIsFront()) {
        pivot.setWantedFlip(PivotFlip.FRONT);
      } else {
        pivot.setWantedFlip(PivotFlip.BACK);
      }
      if (elevator.getElevatorPosition() > Constants.SetPoints.ElevatorPosition.kAUTOL4.meters - 25.0 / 39.37) {
        if (drive.hitSetPointGenerous(drive.getReefL4ClosestSetpoint(drive.getMT2Odometry(), OI.getDriverA())[0],
            drive.getReefL4ClosestSetpoint(drive.getMT2Odometry(), OI.getDriverA())[1],
            drive.getReefL4ClosestSetpoint(drive.getMT2Odometry(), OI.getDriverA())[2])) {
          pivot.setWantedState(PivotState.AUTO_SCORE_L4_SLOW);
        } else {
          pivot.setWantedState(PivotState.AUTO_L4);
        }
      } else {
        pivot.setWantedState(PivotState.DEFAULT);
      }
      twist.setWantedState(TwistState.SIDE);
    }
  }

  public void handleL1PlaceState() {
    manipulator.inL1State = true;
    lights.setWantedState(LightsState.PLACING);
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.L1);
    manipulator.setWantedState(ManipulatorState.DEFAULT);
    pivot.setWantedState(PivotState.L1);
    if (Math.abs(pivot.getPivotPosition()) > 30.0 / 360.0) {
      twist.setWantedState(TwistState.UP);
    }
  }

  public void handleL2PlaceState() {
    lights.setWantedState(LightsState.PLACING);
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.L2);
    manipulator.setWantedState(ManipulatorState.DEFAULT);
    if (OI.driverLB.getAsBoolean()) {
      pivot.setWantedState(PivotState.MANUAL_PLACE);
    } else if (OI.driverRB.getAsBoolean()) {
      pivot.setWantedState(PivotState.MANUAL_RESET);
    } else {
      pivot.setWantedState(PivotState.L23);
    }
    twist.setWantedState(TwistState.SIDE);
  }

  public void handleL3PlaceState() {
    lights.setWantedState(LightsState.PLACING);
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.L3);
    manipulator.setWantedState(ManipulatorState.DEFAULT);
    if (OI.driverLB.getAsBoolean()) {
      pivot.setWantedState(PivotState.MANUAL_PLACE);
    } else if (OI.driverRB.getAsBoolean()) {
      pivot.setWantedState(PivotState.MANUAL_RESET);
    } else {
      pivot.setWantedState(PivotState.L23);
    }
    twist.setWantedState(TwistState.SIDE);
  }

  public void handleL4PlaceState() {
    lights.setWantedState(LightsState.PLACING);
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.L4);
    manipulator.setWantedState(ManipulatorState.DEFAULT);
    if (OI.driverLB.getAsBoolean()) {
      pivot.setWantedState(PivotState.MANUAL_PLACE);
    } else if (OI.driverRB.getAsBoolean()) {
      pivot.setWantedState(PivotState.MANUAL_RESET);
    } else {
      pivot.setWantedState(PivotState.L4);
    }
    twist.setWantedState(TwistState.SIDE);
  }

  public void handleProcessorState() {
    lights.setWantedState(LightsState.PLACING);
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.PROCESSOR);
    manipulator.setWantedState(ManipulatorState.DEFAULT);
    pivot.setWantedState(PivotState.PROCESSOR);
    twist.setWantedState(TwistState.UP);
  }

  public void handleNetState() {
    lights.setWantedState(LightsState.PLACING);
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.NET);
    manipulator.setWantedState(ManipulatorState.DEFAULT);
    pivot.setWantedFlip(PivotFlip.FRONT);
    pivot.setWantedState(PivotState.NET);
    if (elevator.getElevatorPosition() > (Constants.inchesToMeters(10.0))) {
      twist.setWantedState(TwistState.UP);
    }
  }

  public void handleAutoProcessorState() {
    lights.setWantedState(LightsState.PLACING);
    drive.setWantedState(DriveState.PROCESSOR);
    elevator.setWantedState(ElevatorState.PROCESSOR);
    manipulator.setWantedState(ManipulatorState.DEFAULT);
    if (drive.getAutoPlacementSideIsFront()) {
      pivot.setWantedFlip(PivotFlip.FRONT);
      twist.setWantedState(TwistState.UP);
    } else {
      pivot.setWantedFlip(PivotFlip.BACK);
      twist.setWantedState(TwistState.DOWN);
    }
    pivot.setWantedState(PivotState.PROCESSOR);
  }

  public void handleAutoProcessorMoreState() {
    lights.setWantedState(LightsState.PLACING);
    drive.setWantedState(DriveState.PROCESSOR_MORE);
    elevator.setWantedState(ElevatorState.PROCESSOR);
    manipulator.setWantedState(ManipulatorState.DEFAULT);
    if (drive.getAutoPlacementSideIsFront()) {
      pivot.setWantedFlip(PivotFlip.FRONT);
      twist.setWantedState(TwistState.UP);
    } else {
      pivot.setWantedFlip(PivotFlip.BACK);
      twist.setWantedState(TwistState.DOWN);
    }
    pivot.setWantedState(PivotState.PROCESSOR);
  }

  public void handleOutakeDriveState() {
    lights.setWantedState(LightsState.SCORING);
    drive.setWantedState(DriveState.DEFAULT);
    manipulator.setWantedState(ManipulatorState.OUTAKE);
  }

  public void handleAutoNetState() {
    lights.setWantedState(LightsState.PLACING);
    drive.setWantedState(DriveState.NET);
    if (OI.isBlueSide()) {
      if ((drive.getAngleDifferenceDegrees(Math.toDegrees(drive.getMT2OdometryAngle()),
          0.0) < 15.0
          || drive
              .getAngleDifferenceDegrees(Math.toDegrees(drive.getMT2OdometryAngle()),
                  180.0) < 15.0)
          && (Math.abs(drive.getMT2OdometryX() - Constants.Reef.netBlueXM) < 1.5 || Math
              .abs(drive.getMT2OdometryX() - Constants.Reef.netRedXM) < 1.5)) {
        elevator.setWantedState(ElevatorState.NET);
      }
    } else {
      if ((drive.getAngleDifferenceDegrees(Math.toDegrees(drive.getMT2OdometryAngle()),
          0.0) < 15.0
          || drive
              .getAngleDifferenceDegrees(Math.toDegrees(drive.getMT2OdometryAngle()),
                  180.0) < 15.0)
          && (Math.abs(drive.getMT2OdometryX() - Constants.Reef.netBlueXM) < 1.5 || Math
              .abs(drive.getMT2OdometryX() - Constants.Reef.netRedXM) < 1.5)) {
        elevator.setWantedState(ElevatorState.NET);
      }
    }
    manipulator.setWantedState(ManipulatorState.DEFAULT);
    if (!drive.getAutoPlacementSideIsFront()) {
      pivot.setWantedFlip(PivotFlip.BACK);
      if (elevator.getElevatorPosition() > (Constants.inchesToMeters(10.0))) {
        twist.setWantedState(TwistState.DOWN);
      }
    } else {
      pivot.setWantedFlip(PivotFlip.FRONT);
      if (elevator.getElevatorPosition() > (Constants.inchesToMeters(10.0))) {
        twist.setWantedState(TwistState.UP);
      }
    }
    pivot.setWantedState(PivotState.NET);
  }

  public void handleAutoNetStateMore() {
    lights.setWantedState(LightsState.PLACING);
    if (OI.getDriverLTPercent() > 0.2) {
      setWantedState(SuperState.OUTAKE_DRIVE);
    }
    drive.setWantedState(DriveState.NET_MORE);
    if (OI.isBlueSide()) {
      if ((drive.getAngleDifferenceDegrees(Math.toDegrees(drive.getMT2OdometryAngle()),
          0.0) < 15.0
          || drive
              .getAngleDifferenceDegrees(Math.toDegrees(drive.getMT2OdometryAngle()),
                  180.0) < 15.0)
          && (Math.abs(drive.getMT2OdometryX() - Constants.Reef.netBlueXM) < 1.0 || Math
              .abs(drive.getMT2OdometryX() - Constants.Reef.netRedXM) < 1.0)) {
        elevator.setWantedState(ElevatorState.NET);
      }
    } else {
      if ((drive.getAngleDifferenceDegrees(Math.toDegrees(drive.getMT2OdometryAngle()),
          0.0) < 15.0
          || drive
              .getAngleDifferenceDegrees(Math.toDegrees(drive.getMT2OdometryAngle()),
                  180.0) < 15.0)
          && (Math.abs(drive.getMT2OdometryX() - Constants.Reef.netBlueXM) < 1.0 || Math
              .abs(drive.getMT2OdometryX() - Constants.Reef.netRedXM) < 1.0)) {
        elevator.setWantedState(ElevatorState.NET);
      }
    }
    manipulator.setWantedState(ManipulatorState.DEFAULT);
    if (!drive.getAutoPlacementSideIsFront()) {
      pivot.setWantedFlip(PivotFlip.BACK);
      twist.setWantedState(TwistState.DOWN);
    } else {
      pivot.setWantedFlip(PivotFlip.FRONT);
      twist.setWantedState(TwistState.UP);
    }
    pivot.setWantedState(PivotState.NET);
  }

  public void handleFeederAlignState() {
    lights.setWantedState(LightsState.FEEDER);
    drive.setWantedState(DriveState.FEEDER_ALIGN);
    manipulator.setWantedState(ManipulatorState.CORAL_INTAKE);
    if (drive.getFieldSide() == "red") { // red side
      if (drive.getMT2OdometryY() > 4.026) { // redside right feeder (field top right)
        if (!(Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) <= 324
            &&
            Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) >= 144)) {
          if (Math.abs(pivot.getPivotPosition()) > 10.0 / 360.0) {
            twist.setWantedState(TwistState.UP);
          }
          pivot.setWantedFlip(PivotFlip.FRONT);
          pivot.setWantedState(PivotState.FEEDER);
          elevator.setWantedState(ElevatorState.DEFAULT);
        } else { // robot back side redside left feeder (fieldside top right)
          // if (elevator.getElevatorPosition() <= 13 / 39.37 && pivot.getPivotPosition()
          // > -0.08) {
          // elevator.setWantedState(ElevatorState.OVER);
          // } else if (elevator.getElevatorPosition() >= 13 / 39.37 &&
          // pivot.getPivotPosition() > -0.08) {
          // twist.setWantedState(TwistState.DOWN);
          // elevator.setWantedState(ElevatorState.OVER);
          // pivot.setWantedFlip(PivotFlip.BACK);
          // pivot.setWantedState(PivotState.FEEDER);
          // } else {
          elevator.setWantedState(ElevatorState.FEEDER_INTAKE);
          if (Math.abs(pivot.getPivotPosition()) > 10.0 / 360.0) {
            twist.setWantedState(TwistState.DOWN);
          }
          pivot.setWantedFlip(PivotFlip.BACK);
          pivot.setWantedState(PivotState.FEEDER);
          // }
        }
      } else { // redside left feeder (fieldside bottom right)
        if ((Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) <= 36
            &&
            Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) >= 0)
            ||
            (Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) <= 360
                &&
                Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) >= 216)) {
          if (Math.abs(pivot.getPivotPosition()) > 10.0 / 360.0) {
            twist.setWantedState(TwistState.UP);
          }
          pivot.setWantedFlip(PivotFlip.FRONT);
          pivot.setWantedState(PivotState.FEEDER);
          elevator.setWantedState(ElevatorState.DEFAULT);
        } else { // robot back side redside left (fieldside bottom right)
          // if (elevator.getElevatorPosition() <= 13 / 39.37 && pivot.getPivotPosition()
          // > -0.08) {
          // elevator.setWantedState(ElevatorState.OVER);
          // } else if (elevator.getElevatorPosition() >= 13 / 39.37 &&
          // pivot.getPivotPosition() > -0.08) {
          // twist.setWantedState(TwistState.DOWN);
          // elevator.setWantedState(ElevatorState.OVER);
          // pivot.setWantedFlip(PivotFlip.BACK);
          // pivot.setWantedState(PivotState.FEEDER);
          // } else {
          elevator.setWantedState(ElevatorState.FEEDER_INTAKE);
          if (Math.abs(pivot.getPivotPosition()) > 10.0 / 360.0) {
            twist.setWantedState(TwistState.DOWN);
          }
          pivot.setWantedFlip(PivotFlip.BACK);
          pivot.setWantedState(PivotState.FEEDER);
          // }
        }
      }
    } else { // blue side
      if (drive.getMT2OdometryY() < 4.026) { // blue side right feeder (fieldside bottom left)
        if ((Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) <= 324
            &&
            Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) >= 144)) {
          if (Math.abs(pivot.getPivotPosition()) > 10.0 / 360.0) {
            twist.setWantedState(TwistState.UP);
          } // System.out.println("front");
          pivot.setWantedFlip(PivotFlip.FRONT);
          pivot.setWantedState(PivotState.FEEDER);
          elevator.setWantedState(ElevatorState.DEFAULT);
        } else { // robot back side blueside right (fieldside bottom left)
          // if (elevator.getElevatorPosition() <= 13 / 39.37 && pivot.getPivotPosition()
          // > -0.08) {
          // elevator.setWantedState(ElevatorState.OVER);
          // System.out.println("part 1");
          // } else if (elevator.getElevatorPosition() >= 13 / 39.37 &&
          // pivot.getPivotPosition() > -0.08) {
          // twist.setWantedState(TwistState.DOWN);
          // System.out.println("part 2");
          // elevator.setWantedState(ElevatorState.OVER);
          // pivot.setWantedFlip(PivotFlip.BACK);
          // pivot.setWantedState(PivotState.FEEDER);
          // } else {
          elevator.setWantedState(ElevatorState.FEEDER_INTAKE);
          // System.out.println("part 3");
          if (Math.abs(pivot.getPivotPosition()) > 10.0 / 360.0) {
            twist.setWantedState(TwistState.DOWN);
          }
          pivot.setWantedFlip(PivotFlip.BACK);
          pivot.setWantedState(PivotState.FEEDER);
          // }
        }
      } else { // blue side left feeder (fieldside top left)
        if (!((Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) <= 36
            &&
            Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) >= 0)
            ||
            (Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) <= 360
                &&
                Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) >= 216))) {
          if (Math.abs(pivot.getPivotPosition()) > 10.0 / 360.0) {
            twist.setWantedState(TwistState.UP);
          }
          pivot.setWantedFlip(PivotFlip.FRONT);
          pivot.setWantedState(PivotState.FEEDER);
          elevator.setWantedState(ElevatorState.DEFAULT);
        } else { // robot back side blueside left (fieldside top left)
          // if (elevator.getElevatorPosition() <= 13 / 39.37 && pivot.getPivotPosition()
          // > -0.08) {
          // elevator.setWantedState(ElevatorState.OVER);
          // } else if (elevator.getElevatorPosition() >= 13 / 39.37 &&
          // pivot.getPivotPosition() > -0.08) {
          // twist.setWantedState(TwistState.DOWN);
          // elevator.setWantedState(ElevatorState.OVER);
          // pivot.setWantedFlip(PivotFlip.BACK);
          // pivot.setWantedState(PivotState.FEEDER);
          // } else {
          elevator.setWantedState(ElevatorState.FEEDER_INTAKE);
          if (Math.abs(pivot.getPivotPosition()) > 10.0 / 360.0) {
            twist.setWantedState(TwistState.DOWN);
          }
          pivot.setWantedFlip(PivotFlip.BACK);
          pivot.setWantedState(PivotState.FEEDER);
          // }
        }
      }
    }
  }

  public void handleFeederState() {
    lights.setWantedState(LightsState.FEEDER);
    drive.setWantedState(DriveState.DEFAULT);
    manipulator.setWantedState(ManipulatorState.CORAL_INTAKE);
    if (Math.abs(pivot.getPivotPosition()) > 10.0 / 360.0) {
      twist.setWantedState(TwistState.UP);
    }
    pivot.setWantedFlip(PivotFlip.FRONT);
    pivot.setWantedState(PivotState.FEEDER);
    elevator.setWantedState(ElevatorState.FEEDER_INTAKE);
  }

  public void handleFeederAutoState() { // function for an actual field, comment
    // out the function above when running on
    // an actual field
    lights.setWantedState(LightsState.FEEDER);
    drive.setWantedState(DriveState.FEEDER);
    manipulator.setWantedState(ManipulatorState.CORAL_INTAKE);
    if (drive.getFieldSide() == "red") { // red side
      if (drive.getMT2OdometryY() > 4.026) { // redside right feeder (field top right)
        if (!(Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) <= 324
            &&
            Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) >= 144)) {
          if (Math.abs(pivot.getPivotPosition()) > 10.0 / 360.0) {
            twist.setWantedState(TwistState.UP);
          }
          pivot.setWantedFlip(PivotFlip.FRONT);
          pivot.setWantedState(PivotState.FEEDER);
          elevator.setWantedState(ElevatorState.DEFAULT);
        } else { // robot back side redside left feeder (fieldside top right)
          // if (elevator.getElevatorPosition() <= 13 / 39.37 && pivot.getPivotPosition()
          // > -0.08) {
          // elevator.setWantedState(ElevatorState.OVER);
          // } else if (elevator.getElevatorPosition() >= 13 / 39.37 &&
          // pivot.getPivotPosition() > -0.08) {
          // twist.setWantedState(TwistState.DOWN);
          // elevator.setWantedState(ElevatorState.OVER);
          // pivot.setWantedFlip(PivotFlip.BACK);
          // pivot.setWantedState(PivotState.FEEDER);
          // } else {
          elevator.setWantedState(ElevatorState.FEEDER_INTAKE);
          if (Math.abs(pivot.getPivotPosition()) > 10.0 / 360.0) {
            twist.setWantedState(TwistState.DOWN);
          }
          pivot.setWantedFlip(PivotFlip.BACK);
          pivot.setWantedState(PivotState.FEEDER);
          // }
        }
      } else { // redside left feeder (fieldside bottom right)
        if ((Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) <= 36
            &&
            Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) >= 0)
            ||
            (Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) <= 360
                &&
                Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) >= 216)) {
          if (Math.abs(pivot.getPivotPosition()) > 10.0 / 360.0) {
            twist.setWantedState(TwistState.UP);
          }
          pivot.setWantedFlip(PivotFlip.FRONT);
          pivot.setWantedState(PivotState.FEEDER);
          elevator.setWantedState(ElevatorState.DEFAULT);
        } else { // robot back side redside left (fieldside bottom right)
          // if (elevator.getElevatorPosition() <= 13 / 39.37 && pivot.getPivotPosition()
          // > -0.08) {
          // elevator.setWantedState(ElevatorState.OVER);
          // } else if (elevator.getElevatorPosition() >= 13 / 39.37 &&
          // pivot.getPivotPosition() > -0.08) {
          // twist.setWantedState(TwistState.DOWN);
          // elevator.setWantedState(ElevatorState.OVER);
          // pivot.setWantedFlip(PivotFlip.BACK);
          // pivot.setWantedState(PivotState.FEEDER);
          // } else {
          elevator.setWantedState(ElevatorState.FEEDER_INTAKE);
          if (Math.abs(pivot.getPivotPosition()) > 10.0 / 360.0) {
            twist.setWantedState(TwistState.DOWN);
          }
          pivot.setWantedFlip(PivotFlip.BACK);
          pivot.setWantedState(PivotState.FEEDER);
          // }
        }
      }
    } else { // blue side
      if (drive.getMT2OdometryY() < 4.026) { // blue side right feeder (fieldside bottom left)
        if ((Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) <= 324
            &&
            Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) >= 144)) {
          if (Math.abs(pivot.getPivotPosition()) > 10.0 / 360.0) {
            twist.setWantedState(TwistState.UP);
          } // System.out.println("front");
          pivot.setWantedFlip(PivotFlip.FRONT);
          pivot.setWantedState(PivotState.FEEDER);
          elevator.setWantedState(ElevatorState.DEFAULT);
        } else { // robot back side blueside right (fieldside bottom left)
          // if (elevator.getElevatorPosition() <= 13 / 39.37 && pivot.getPivotPosition()
          // > -0.08) {
          // elevator.setWantedState(ElevatorState.OVER);
          // System.out.println("part 1");
          // } else if (elevator.getElevatorPosition() >= 13 / 39.37 &&
          // pivot.getPivotPosition() > -0.08) {
          // twist.setWantedState(TwistState.DOWN);
          // System.out.println("part 2");
          // elevator.setWantedState(ElevatorState.OVER);
          // pivot.setWantedFlip(PivotFlip.BACK);
          // pivot.setWantedState(PivotState.FEEDER);
          // } else {
          elevator.setWantedState(ElevatorState.FEEDER_INTAKE);
          // System.out.println("part 3");
          if (Math.abs(pivot.getPivotPosition()) > 10.0 / 360.0) {
            twist.setWantedState(TwistState.DOWN);
          }
          pivot.setWantedFlip(PivotFlip.BACK);
          pivot.setWantedState(PivotState.FEEDER);
          // }
        }
      } else { // blue side left feeder (fieldside top left)
        if (!((Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) <= 36
            &&
            Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) >= 0)
            ||
            (Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) <= 360
                &&
                Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) >= 216))) {
          if (Math.abs(pivot.getPivotPosition()) > 10.0 / 360.0) {
            twist.setWantedState(TwistState.UP);
          }
          pivot.setWantedFlip(PivotFlip.FRONT);
          pivot.setWantedState(PivotState.FEEDER);
          elevator.setWantedState(ElevatorState.DEFAULT);
        } else { // robot back side blueside left (fieldside top left)
          // if (elevator.getElevatorPosition() <= 13 / 39.37 && pivot.getPivotPosition()
          // > -0.08) {
          // elevator.setWantedState(ElevatorState.OVER);
          // } else if (elevator.getElevatorPosition() >= 13 / 39.37 &&
          // pivot.getPivotPosition() > -0.08) {
          // twist.setWantedState(TwistState.DOWN);
          // elevator.setWantedState(ElevatorState.OVER);
          // pivot.setWantedFlip(PivotFlip.BACK);
          // pivot.setWantedState(PivotState.FEEDER);
          // } else {
          elevator.setWantedState(ElevatorState.FEEDER_INTAKE);
          if (Math.abs(pivot.getPivotPosition()) > 10.0 / 360.0) {
            twist.setWantedState(TwistState.DOWN);
          }
          pivot.setWantedFlip(PivotFlip.BACK);
          pivot.setWantedState(PivotState.FEEDER);
          // }
        }
      }
    }
  }

  /*
   */
  public void handleGroundCoralPickupFrontState() {
    lights.setWantedState(LightsState.INTAKING);
    drive.setWantedState(DriveState.DEFAULT);
    intake.setWantedState(IntakeState.INTAKING);
    // pivot.setWantedState(PivotState.HANDOFF);
    // manipulator.setWantedState(ManipulatorState.DEFAULT);
    // elevator.setWantedState(ElevatorState.PREHANDOFF);
    // if (pivot.getPivotPosition() > 15.0 / 360.0) {
    // twist.setWantedState(TwistState.UP);
    // }
  }

  public void handleGroundCoralPickupBackState() {
    lights.setWantedState(LightsState.INTAKING);
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.GROUND_CORAL_INTAKE);
    manipulator.setWantedState(ManipulatorState.CORAL_INTAKE);
    pivot.setWantedState(PivotState.GROUND_CORAL_BACK);
    if (pivot.getPivotPosition() < -15.0 / 360.0) {
      twist.setWantedState(TwistState.DOWN);
    }
  }

  public void handleAutoGroundCoralPickupFrontState() {
    lights.setWantedState(LightsState.INTAKING);
    if (Math.abs(intake.getPosition()) > 0.25) {
      drive.setWantedState(DriveState.PIECE_PICKUP);
    } else {
      if (DriverStation.isAutonomousEnabled()) {
        drive.setWantedState(DriveState.STOP);
      } else {
        drive.setWantedState(DriveState.DEFAULT);
      }
    }
    intake.setWantedState(IntakeState.INTAKING);
    pivot.setWantedState(PivotState.HANDOFF);
    manipulator.setWantedState(ManipulatorState.DEFAULT);
    elevator.setWantedState(ElevatorState.PREHANDOFF);
    if (pivot.getPivotPosition() > 15.0 / 360.0) {
      twist.setWantedState(TwistState.UP);
    }
  }

  public void handleAutoGroundCoralPickupBackState() {
    lights.setWantedState(LightsState.INTAKING);
    if (Math.abs(pivot.getPivotPosition()) > 0.25) {
      drive.setWantedState(DriveState.PIECE_PICKUP);
    } else {
      if (DriverStation.isAutonomousEnabled()) {
        drive.setWantedState(DriveState.STOP);
      } else {
        drive.setWantedState(DriveState.DEFAULT);
      }
    }
    elevator.setWantedState(ElevatorState.GROUND_CORAL_INTAKE);
    manipulator.setWantedState(ManipulatorState.CORAL_INTAKE);
    pivot.setWantedState(PivotState.GROUND_CORAL_BACK);
    if (pivot.getPivotPosition() < -15.0 / 360.0) {
      twist.setWantedState(TwistState.DOWN);
    }
  }

  public void handleGroundAlgaePickupFrontState() {
    lights.setWantedState(LightsState.INTAKING);
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.GROUND_ALGAE_INTAKE);
    manipulator.setWantedState(ManipulatorState.ALGAE_INTAKE);
    pivot.setWantedFlip(PivotFlip.FRONT);
    pivot.setWantedState(PivotState.GROUND_ALGAE);
    if (Math.abs(pivot.getPivotPosition()) > 10.0 / 360.0) {
      twist.setWantedState(TwistState.UP);
    }
  }

  public void handleGroundAlgaePickupBackState() {
    lights.setWantedState(LightsState.INTAKING);
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.GROUND_ALGAE_INTAKE);
    manipulator.setWantedState(ManipulatorState.ALGAE_INTAKE);
    pivot.setWantedFlip(PivotFlip.BACK);
    pivot.setWantedState(PivotState.GROUND_ALGAE);
    if (Math.abs(pivot.getPivotPosition()) < 10.0 / 360.0) {
      twist.setWantedState(TwistState.DOWN);
    }
  }

  public void handleL2AlgaePickupState() {
    lights.setWantedState(LightsState.FEEDER);
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.L2_ALGAE);
    manipulator.setWantedState(ManipulatorState.ALGAE_INTAKE);
    pivot.setWantedFlip(PivotFlip.FRONT);
    pivot.setWantedState(PivotState.REEF_ALGAE);
    twist.setWantedState(TwistState.DOWN);
  }

  public void handleL3AlgaePickupState() {
    lights.setWantedState(LightsState.FEEDER);
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.L3_ALGAE);
    manipulator.setWantedState(ManipulatorState.ALGAE_INTAKE);
    pivot.setWantedFlip(PivotFlip.FRONT);
    pivot.setWantedState(PivotState.REEF_ALGAE);
    twist.setWantedState(TwistState.DOWN);
  }

  public void handleAutoAlgaePickupState() {
    lights.setWantedState(LightsState.PLACING);
    drive.setWantedState(DriveState.ALGAE);
    manipulator.setWantedState(ManipulatorState.ALGAE_INTAKE);
    if (drive.isGoingForL3Algae()) {
      elevator.setWantedState(ElevatorState.L3_ALGAE);
    } else {
      elevator.setWantedState(ElevatorState.L2_ALGAE);
    }
    if (drive.getAutoPlacementSideIsFront()) {
      twist.setWantedState(TwistState.DOWN);
      pivot.setWantedFlip(PivotFlip.FRONT);
    } else {
      twist.setWantedState(TwistState.UP);
      pivot.setWantedFlip(PivotFlip.BACK);
    }
    pivot.setWantedState(PivotState.REEF_ALGAE);
  }

  // private double algaePickupTime = Timer.getFPGATimestamp();
  // private boolean finishedAlgae1 = false;
  // private boolean finishedAlgae2 = false;

  public void handleAutoAlgaePickupMoreState() {
    lights.setWantedState(LightsState.FEEDER);
    if (manipulator.getArmItem() == ArmItem.NONE
        && Math.hypot(OI.getDriverLeftX(), OI.getDriverLeftY()) < 0.1 && Math
            .hypot(OI.getDriverRightX(), OI.getDriverRightY()) < 0.1) {
      drive.setWantedState(DriveState.ALGAE_MORE);
    } else {
      // drive.setWantedState(DriveState.DEFAULT);
      // lights.setWantedState(LightsState.SCORING);
      setWantedState(SuperState.AUTO_ALGAE_PICKUP_MORE_MORE);
    }
  }

  public void handleAutoAlgaePickupMoreMoreState() {
    lights.setWantedState(LightsState.INTAKING);
    if (!drive.hitSetPointGenerous(drive.getAlgaeMoreMoreClosestSetpoint(drive.getMT2Odometry())[0],
        drive.getAlgaeMoreMoreClosestSetpoint(drive
            .getMT2Odometry())[1],
        drive.getAlgaeMoreMoreClosestSetpoint(drive
            .getMT2Odometry())[2])
        && Math.hypot(OI.getDriverLeftX(), OI.getDriverLeftY()) < 0.1 && Math
            .hypot(OI.getDriverRightX(), OI.getDriverRightY()) < 0.1) {
      drive.setWantedState(DriveState.ALGAE_MORE_MORE);
    } else {
      // drive.setWantedState(DriveState.DEFAULT);
      // lights.setWantedState(LightsState.SCORING);
      setWantedState(SuperState.DEFAULT_DRIVE);
    }
  }

  public void handleDeployClimberState() {
    lights.setWantedState(LightsState.CLIMB_DEPLOY);
    climber.setWantedState(ClimbState.EXTENDING);
    pivot.setWantedState(PivotState.CLIMB);
    // peripherals.setBackCamPipline(1);
  }

  public void handleClimbState() {
    lights.setWantedState(LightsState.CLIMB);
    climber.setWantedState(ClimbState.RETRACTING);
    pivot.setWantedState(PivotState.CLIMB);
  }

  public void handleClimberIdleState() {
    lights.setWantedState(LightsState.CLIMB_IDLE);
    climber.setWantedState(ClimbState.IDLE);
    pivot.setWantedState(PivotState.CLIMB);
  }

  public void handleOutakeState() {
    lights.setWantedState(LightsState.SCORING);
    manipulator.setWantedState(ManipulatorState.OUTAKE);
  }

  public void handleAutoL1ScoreState() {
    manipulator.inL1State = true;
    // lights.setWantedState(LightsState.SCORING);
    // drive.setWantedState(DriveState.DEFAULT);
    // intake.setWantedState(IntakeState.OUTAKE);
    // lights.setWantedState(LightsState.SCORING);
    // pivot.setWantedState(PivotState.AUTO_SCORE_L1);

    if (Math.hypot(OI.getDriverLeftX(), OI.getDriverLeftY()) > 0.25 || Math.hypot(OI.getDriverLeftX(),
        OI.getDriverLeftY()) > 0.25) {
      drive.setWantedState(DriveState.DEFAULT);
      // intake.setWantedState(IntakeState.OUTAKE);
    } else {
      // if (drive.hitSetPoint(drive.getL1ReefClosestSetpoint(drive.getMT2Odometry(),
      // OI.getDriverA())[0],
      // drive.getL1ReefClosestSetpoint(drive.getMT2Odometry(), OI.getDriverA())[1],
      // drive.getL1ReefClosestSetpoint(drive.getMT2Odometry(), OI.getDriverA())[2]))
      // {
      manipulator.setWantedState(ManipulatorState.OUTAKE);
      // } else {
      // intake.setWantedState(IntakeState.OFF);
      // }
      if (manipulator.getArmItem() == ArmItem.NONE && !DriverStation.isAutonomousEnabled()) {
        drive.setWantedState(DriveState.REEF_MORE);
        // elevator.setWantedState(ElevatorState.DEFAULT);
      } else {
        // drive.setWantedState(DriveState.DEFAULT);
      }
    }
  }

  // public void handleAutoL2ScoreState() {
  // lights.setWantedState(LightsState.SCORING);
  // drive.setWantedState(DriveState.DEFAULT);
  // pivot.setWantedState(PivotState.AUTO_SCORE_L2);
  // // if (Math.hypot((drive.getMT2OdometryX() -
  // // drive.getReefClosestSetpoint(drive.getMT2Odometry())[0]),
  // // (drive.getMT2OdometryY() -
  // // drive.getReefClosestSetpoint(drive.getMT2Odometry())[1])) > 2.0 / 39.37) {
  // // intake.setWantedState(IntakeState.OUTAKE);
  // // } else {
  // // intake.setWantedState(IntakeState.OFF);
  // // }
  // if (Math.hypot(OI.getDriverLeftX(), OI.getDriverLeftY()) > 0.1 ||
  // Math.hypot(OI.getDriverLeftX(),
  // OI.getDriverLeftY()) > 0.1) {
  // intake.setWantedState(IntakeState.OUTAKE);
  // } else {
  // intake.setWantedState(IntakeState.OFF);
  // }
  // }

  public void handleAutoL2ScoreState() {
    lights.setWantedState(LightsState.SCORING);
    if (elevator.getElevatorPosition() > Constants.SetPoints.ElevatorPosition.kAUTOL2.meters - 15.0 / 39.37) {
      pivot.setWantedState(PivotState.AUTO_SCORE_L2);
    } else {
      pivot.setWantedState(PivotState.DEFAULT);
    }
    // if (Math.hypot((drive.getMT2OdometryX() -
    // drive.getReefL4ClosestSetpoint(drive.getMT2Odometry())[0]),
    // (drive.getMT2OdometryY() -
    // drive.getReefL4ClosestSetpoint(drive.getMT2Odometry())[1])) > 2.0 / 39.37) {
    // intake.setWantedState(IntakeState.OUTAKE);
    // } else {
    // intake.setWantedState(IntakeState.OFF);
    // }
    if (Math.hypot(OI.getDriverLeftX(), OI.getDriverLeftY()) > 0.25 || Math.hypot(OI.getDriverLeftX(),
        OI.getDriverLeftY()) > 0.25) {
      drive.setWantedState(DriveState.DEFAULT);
      manipulator.setWantedState(ManipulatorState.OUTAKE);
    } else {
      if (Math.abs(pivot.getPivotPosition()) > Constants.SetPoints.PivotPosition.kAUTOL2SCORE.rotations
          - 15.0 / 360.0 && !DriverStation.isAutonomousEnabled()) {
        manipulator.setWantedState(ManipulatorState.OUTAKE);
      } else {
        manipulator.setWantedState(ManipulatorState.OFF);
      }
      if (Math.abs(pivot.getPivotPosition()) > Constants.SetPoints.PivotPosition.kAUTOL2SCORE.rotations
          - 15.0 / 360.0 && !DriverStation.isAutonomousEnabled()) {
        drive.setWantedState(DriveState.REEF_MORE);
      } else {
        drive.setWantedState(DriveState.DEFAULT);
      }
      if (Math.abs(pivot.getPivotPosition()) > Constants.SetPoints.PivotPosition.kAUTOL2SCORE.rotations
          - 15.0 / 360.0) {
        elevator.setWantedState(ElevatorState.AUTO_L2);
      }
    }
  }

  // public void handleAutoL3ScoreState() {
  // drive.setWantedState(DriveState.DEFAULT);
  // pivot.setWantedState(PivotState.AUTO_SCORE_L23);
  // }

  // public void handleAutoL3ScoreState() {
  // // lights.setWantedState(LightsState.SCORING);
  // //
  // elevator.updateDistanceFromL23DriveSetpoint(drive.getDistanceFromL23Setpoint());
  // // drive.setWantedState(DriveState.SCORE_L23);
  // // elevator.setWantedState(ElevatorState.AUTO_SCORE_L3);
  // lights.setWantedState(LightsState.SCORING);
  // drive.setWantedState(DriveState.DEFAULT);
  // pivot.setWantedState(PivotState.AUTO_SCORE_L3);
  // // if (Math.hypot((drive.getMT2OdometryX() -
  // // drive.getReefClosestSetpoint(drive.getMT2Odometry())[0]),
  // // (drive.getMT2OdometryY() -
  // // drive.getReefClosestSetpoint(drive.getMT2Odometry())[1])) > 2.0 / 39.37) {
  // // intake.setWantedState(IntakeState.OUTAKE);
  // // } else {
  // // intake.setWantedState(IntakeState.OFF);
  // // }
  // if (Math.hypot(OI.getDriverLeftX(), OI.getDriverLeftY()) > 0.1 ||
  // Math.hypot(OI.getDriverLeftX(),
  // OI.getDriverLeftY()) > 0.1) {
  // intake.setWantedState(IntakeState.OUTAKE);
  // } else {
  // intake.setWantedState(IntakeState.OFF);
  // }
  // }

  public void handleAutoL3ScoreState() {
    lights.setWantedState(LightsState.SCORING);
    if (elevator.getElevatorPosition() > Constants.SetPoints.ElevatorPosition.kAUTOL3.meters - 15.0 / 39.37) {
      pivot.setWantedState(PivotState.AUTO_SCORE_L3);
    } else {
      pivot.setWantedState(PivotState.DEFAULT);
    }
    // if (Math.hypot((drive.getMT2OdometryX() -
    // drive.getReefL4ClosestSetpoint(drive.getMT2Odometry())[0]),
    // (drive.getMT2OdometryY() -
    // drive.getReefL4ClosestSetpoint(drive.getMT2Odometry())[1])) > 2.0 / 39.37) {
    // intake.setWantedState(IntakeState.OUTAKE);
    // } else {
    // intake.setWantedState(IntakeState.OFF);
    // }
    if (Math.hypot(OI.getDriverLeftX(), OI.getDriverLeftY()) > 0.25 || Math.hypot(OI.getDriverLeftX(),
        OI.getDriverLeftY()) > 0.25) {
      drive.setWantedState(DriveState.DEFAULT);
      manipulator.setWantedState(ManipulatorState.OUTAKE);
    } else {
      if (Math.abs(pivot.getPivotPosition()) > Constants.SetPoints.PivotPosition.kAUTOL3SCORE.rotations
          - 20.0 / 360.0 && !DriverStation.isAutonomousEnabled()) {
        manipulator.setWantedState(ManipulatorState.OUTAKE);
      } else {
        manipulator.setWantedState(ManipulatorState.OFF);
      }
      if (Math.abs(pivot.getPivotPosition()) > Constants.SetPoints.PivotPosition.kAUTOL3SCORE.rotations
          - 20.0 / 360.0 && !DriverStation.isAutonomousEnabled()) {
        drive.setWantedState(DriveState.REEF_MORE);
        elevator.setWantedState(ElevatorState.AUTO_L3);
      } else {
        drive.setWantedState(DriveState.DEFAULT);
      }
    }
  }

  public void handleAutoL3ScoreMoreState() {
    lights.setWantedState(LightsState.SCORING);
    drive.setWantedState(DriveState.DEFAULT);
    pivot.setWantedState(PivotState.AUTO_SCORE_L3);
    elevator.setWantedState(ElevatorState.AUTO_SCORE_MORE_L3);
  }

  private boolean hasPlaced = false;

  public void handleAutoL4ScoreState() {
    lights.setWantedState(LightsState.SCORING);
    if (OI.operatorLT.getAsBoolean() && OI.operatorRT.getAsBoolean()) {
      drive.setWantedState(DriveState.DEFAULT);
    } else {
      if (elevator.getElevatorPosition() > Constants.SetPoints.ElevatorPosition.kAUTOL4.meters - 15.0 / 39.37) {
        pivot.setWantedState(PivotState.AUTO_SCORE_L4);
      } else {
        pivot.setWantedState(PivotState.DEFAULT);
      }
      // if (Math.hypot((drive.getMT2OdometryX() -
      // drive.getReefL4ClosestSetpoint(drive.getMT2Odometry())[0]),
      // (drive.getMT2OdometryY() -
      // drive.getReefL4ClosestSetpoint(drive.getMT2Odometry())[1])) > 2.0 / 39.37) {
      // intake.setWantedState(IntakeState.OUTAKE);
      // } else {
      // intake.setWantedState(IntakeState.OFF);
      // }

      if (drive.hitSetPointGenerous(drive.getReefMoreClosestSetpoint(drive.getMT2Odometry())[0],
          drive.getReefMoreClosestSetpoint(drive.getMT2Odometry())[1],
          drive.getReefMoreClosestSetpoint(drive.getMT2Odometry())[2])) {
        setWantedState(SuperState.DEFAULT);
      }

      if (Math.hypot(OI.getDriverLeftX(), OI.getDriverLeftY()) > 0.25 || Math.hypot(OI.getDriverLeftX(),
          OI.getDriverLeftY()) > 0.25) {
        drive.setWantedState(DriveState.DEFAULT);
        manipulator.setWantedState(ManipulatorState.OUTAKE);
      } else {
        if (Math.abs(pivot.getPivotPosition()) > Constants.SetPoints.PivotPosition.kAUTOL4SCORE.rotations
            - 20.0 / 360.0 && !DriverStation.isAutonomousEnabled()) {
          hasPlaced = true;
          drive.setWantedState(DriveState.REEF_MORE);
          manipulator.setWantedState(ManipulatorState.OUTAKE);
          elevator.setWantedState(ElevatorState.DEFAULT);
        } else {
          if (hasPlaced) {
            drive.setWantedState(DriveState.DEFAULT);
          } else {
            drive.setWantedState(DriveState.L4_REEF);
          }
          manipulator.setWantedState(ManipulatorState.OFF);
        }
      }
    }
  }

  public void handleScoreL1State() {
    lights.setWantedState(LightsState.SCORING);
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.SCORE_L1);
    manipulator.setWantedState(ManipulatorState.OUTAKE);
    pivot.setWantedState(PivotState.SCORE_L1);
  }

  public void handleScoreL2State() {
    lights.setWantedState(LightsState.SCORING);
    drive.setWantedState(DriveState.DEFAULT);
    // if (Math.hypot((drive.getMT2OdometryX() -
    // drive.getReefClosestSetpoint(drive.getMT2Odometry())[0]),
    // (drive.getMT2OdometryY() -
    // drive.getReefClosestSetpoint(drive.getMT2Odometry())[1])) > 2.0 / 39.37) {
    // intake.setWantedState(IntakeState.OUTAKE);
    // } else {
    if (Math.hypot(OI.getDriverLeftX(), OI.getDriverLeftY()) > 0.25 || Math.hypot(OI.getDriverLeftX(),
        OI.getDriverLeftY()) > 0.25) {
      manipulator.setWantedState(ManipulatorState.OUTAKE);
    } else {
      manipulator.setWantedState(ManipulatorState.OFF);
    }
    // }
    pivot.setWantedState(PivotState.SCORE_L23);
  }

  public void handleScoreL3State() {
    lights.setWantedState(LightsState.SCORING);
    drive.setWantedState(DriveState.DEFAULT);
    // if (Math.hypot((drive.getMT2OdometryX() -
    // drive.getReefClosestSetpoint(drive.getMT2Odometry())[0]),
    // (drive.getMT2OdometryY() -
    // drive.getReefClosestSetpoint(drive.getMT2Odometry())[1])) > 2.0 / 39.37) {
    // intake.setWantedState(IntakeState.OUTAKE);
    // } else {
    if (Math.hypot(OI.getDriverLeftX(), OI.getDriverLeftY()) > 0.25 || Math.hypot(OI.getDriverLeftX(),
        OI.getDriverLeftY()) > 0.25) {
      manipulator.setWantedState(ManipulatorState.OUTAKE);
    } else {
      manipulator.setWantedState(ManipulatorState.OFF);
    } // }
    pivot.setWantedState(PivotState.SCORE_L23);
  }

  public void handleScoreL4State() {
    lights.setWantedState(LightsState.SCORING);
    drive.setWantedState(DriveState.DEFAULT);
    // elevator.setWantedState(ElevatorState.SCORE_L4);
    pivot.setWantedState(PivotState.SCORE_L4);
    // if (Math.hypot((drive.getMT2OdometryX() -
    // drive.getReefClosestSetpoint(drive.getMT2Odometry())[0]),
    // (drive.getMT2OdometryY() -
    // drive.getReefClosestSetpoint(drive.getMT2Odometry())[1])) > 2.0 / 39.37) {
    // intake.setWantedState(IntakeState.OUTAKE);
    // } else {
    if (Math.hypot(OI.getDriverLeftX(), OI.getDriverLeftY()) > 0.25 || Math.hypot(OI.getDriverLeftX(),
        OI.getDriverLeftY()) > 0.25) {
      manipulator.setWantedState(ManipulatorState.OUTAKE);
    } else {
      manipulator.setWantedState(ManipulatorState.OFF);
    } // }
  }

  public void handleIdleState() {
    lights.setWantedState(LightsState.DEFAULT);
    drive.setWantedState(DriveState.IDLE);
    // pivot.setWantedFlip(PivotFlip.FRONT);
    // if (twist.getTwistPosition() < 45) {
    elevator.setWantedState(ElevatorState.DEFAULT);
    // } else {
    // elevator.setWantedState(ElevatorState.GROUND_CORAL_INTAKE);
    // }
    manipulator.setWantedState(ManipulatorState.DEFAULT);
    if (Math.abs(twist.getTwistPosition()) < 15.0) {
      pivot.setWantedState(PivotState.DEFAULT);
    }
    twist.setWantedState(TwistState.SIDE);
    climber.setWantedState(ClimbState.IDLE);
  }

  public void handleOutakeIdleState() {
    manipulator.setWantedState(ManipulatorState.OUTAKE);
    if (!outakeIdleInit) {
      outakeIdleInitTime = Timer.getFPGATimestamp();
      outakeIdleInit = true;
    }
    if (Timer.getFPGATimestamp() - outakeIdleInitTime > 0.25) {
      lights.setWantedState(LightsState.DEFAULT);
      drive.setWantedState(DriveState.IDLE);
      // pivot.setWantedFlip(PivotFlip.FRONT);
      if (twist.getTwistPosition() < 45) {
        elevator.setWantedState(ElevatorState.DEFAULT);
      } else {
        elevator.setWantedState(ElevatorState.GROUND_CORAL_INTAKE);
      }
      if (Math.abs(twist.getTwistPosition()) < 15 && elevator.getElevatorPosition() < Constants.inchesToMeters(48.0)) {
        pivot.setWantedState(PivotState.DEFAULT);
      }
      twist.setWantedState(TwistState.SIDE);
      climber.setWantedState(ClimbState.IDLE);
    }
  }

  public void handleManualPlaceState() {
    pivot.setWantedState(PivotState.MANUAL_PLACE);
  }

  public void handleManualResetState() {
    pivot.setWantedState(PivotState.MANUAL_RESET);
  }

  public void handleAutoFeederState() {
    lights.setWantedState(LightsState.FEEDER);
    drive.setWantedState(DriveState.AUTO_FEEDER);
    manipulator.setWantedState(ManipulatorState.CORAL_INTAKE);
    if (drive.getFieldSide() == "red") { // red side
      if (drive.getMT2OdometryY() > 4.026) { // redside right feeder (field top right)
        if (!(Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) <= 324
            &&
            Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) >= 144)) {
          if (Math.abs(pivot.getPivotPosition()) > 10.0 / 360.0) {
            twist.setWantedState(TwistState.UP);
          }
          pivot.setWantedFlip(PivotFlip.FRONT);
          pivot.setWantedState(PivotState.FEEDER);
          elevator.setWantedState(ElevatorState.DEFAULT);
        } else { // robot back side redside left feeder (fieldside top right)
          // if (elevator.getElevatorPosition() <= 13 / 39.37 && pivot.getPivotPosition()
          // > -0.08) {
          // elevator.setWantedState(ElevatorState.OVER);
          // } else if (elevator.getElevatorPosition() >= 13 / 39.37 &&
          // pivot.getPivotPosition() > -0.08) {
          // twist.setWantedState(TwistState.DOWN);
          // elevator.setWantedState(ElevatorState.OVER);
          // pivot.setWantedFlip(PivotFlip.BACK);
          // pivot.setWantedState(PivotState.FEEDER);
          // } else {
          elevator.setWantedState(ElevatorState.FEEDER_INTAKE);
          if (Math.abs(pivot.getPivotPosition()) > 10.0 / 360.0) {
            twist.setWantedState(TwistState.DOWN);
          }
          pivot.setWantedFlip(PivotFlip.BACK);
          pivot.setWantedState(PivotState.FEEDER);
          // }
        }
      } else { // redside left feeder (fieldside bottom right)
        if ((Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) <= 36
            &&
            Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) >= 0)
            ||
            (Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) <= 360
                &&
                Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) >= 216)) {
          if (Math.abs(pivot.getPivotPosition()) > 10.0 / 360.0) {
            twist.setWantedState(TwistState.UP);
          }
          pivot.setWantedFlip(PivotFlip.FRONT);
          pivot.setWantedState(PivotState.FEEDER);
          elevator.setWantedState(ElevatorState.DEFAULT);
        } else { // robot back side redside left (fieldside bottom right)
          // if (elevator.getElevatorPosition() <= 13 / 39.37 && pivot.getPivotPosition()
          // > -0.08) {
          // elevator.setWantedState(ElevatorState.OVER);
          // } else if (elevator.getElevatorPosition() >= 13 / 39.37 &&
          // pivot.getPivotPosition() > -0.08) {
          // twist.setWantedState(TwistState.DOWN);
          // elevator.setWantedState(ElevatorState.OVER);
          // pivot.setWantedFlip(PivotFlip.BACK);
          // pivot.setWantedState(PivotState.FEEDER);
          // } else {
          elevator.setWantedState(ElevatorState.FEEDER_INTAKE);
          if (Math.abs(pivot.getPivotPosition()) > 10.0 / 360.0) {
            twist.setWantedState(TwistState.DOWN);
          }
          pivot.setWantedFlip(PivotFlip.BACK);
          pivot.setWantedState(PivotState.FEEDER);
          // }
        }
      }
    } else { // blue side
      if (drive.getMT2OdometryY() < 4.026) { // blue side right feeder (fieldside bottom left)
        if ((Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) <= 324
            &&
            Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) >= 144)) {
          if (Math.abs(pivot.getPivotPosition()) > 10.0 / 360.0) {
            twist.setWantedState(TwistState.UP);
          } // System.out.println("front");
          pivot.setWantedFlip(PivotFlip.FRONT);
          pivot.setWantedState(PivotState.FEEDER);
          elevator.setWantedState(ElevatorState.DEFAULT);
        } else { // robot back side blueside right (fieldside bottom left)
          // if (elevator.getElevatorPosition() <= 13 / 39.37 && pivot.getPivotPosition()
          // > -0.08) {
          // elevator.setWantedState(ElevatorState.OVER);
          // System.out.println("part 1");
          // } else if (elevator.getElevatorPosition() >= 13 / 39.37 &&
          // pivot.getPivotPosition() > -0.08) {
          // twist.setWantedState(TwistState.DOWN);
          // System.out.println("part 2");
          // elevator.setWantedState(ElevatorState.OVER);
          // pivot.setWantedFlip(PivotFlip.BACK);
          // pivot.setWantedState(PivotState.FEEDER);
          // } else {
          elevator.setWantedState(ElevatorState.FEEDER_INTAKE);
          // System.out.println("part 3");
          if (Math.abs(pivot.getPivotPosition()) > 10.0 / 360.0) {
            twist.setWantedState(TwistState.DOWN);
          }
          pivot.setWantedFlip(PivotFlip.BACK);
          pivot.setWantedState(PivotState.FEEDER);
          // }
        }
      } else { // blue side left feeder (fieldside top left)
        if (!((Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) <= 36
            &&
            Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) >= 0)
            ||
            (Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) <= 360
                &&
                Constants.standardizeAngleDegrees(Math.toDegrees(drive.getMT2OdometryAngle())) >= 216))) {
          if (Math.abs(pivot.getPivotPosition()) > 10.0 / 360.0) {
            twist.setWantedState(TwistState.UP);
          }
          pivot.setWantedFlip(PivotFlip.FRONT);
          pivot.setWantedState(PivotState.FEEDER);
          elevator.setWantedState(ElevatorState.DEFAULT);
        } else { // robot back side blueside left (fieldside top left)
          // if (elevator.getElevatorPosition() <= 13 / 39.37 && pivot.getPivotPosition()
          // > -0.08) {
          // elevator.setWantedState(ElevatorState.OVER);
          // } else if (elevator.getElevatorPosition() >= 13 / 39.37 &&
          // pivot.getPivotPosition() > -0.08) {
          // twist.setWantedState(TwistState.DOWN);
          // elevator.setWantedState(ElevatorState.OVER);
          // pivot.setWantedFlip(PivotFlip.BACK);
          // pivot.setWantedState(PivotState.FEEDER);
          // } else {
          elevator.setWantedState(ElevatorState.FEEDER_INTAKE);
          if (Math.abs(pivot.getPivotPosition()) > 10.0 / 360.0) {
            twist.setWantedState(TwistState.DOWN);
          }
          pivot.setWantedFlip(PivotFlip.BACK);
          pivot.setWantedState(PivotState.FEEDER);
          // }
        }
      }
    }
  }

  public void handleRunClimbBack() {
    if (OI.driverViewButton.getAsBoolean()) {
      climber.setWantedState(ClimbState.RETRACTING);
    } else {
      climber.setWantedState(ClimbState.IDLE);
    }
    pivot.setWantedState(PivotState.CLIMB);
  }

  public void handleAutoClimbState() {
    pivot.setWantedState(PivotState.CLIMB);

    if (!continueClimbing && climber.getPosition() > -400) {
      climber.setWantedState(ClimbState.EXTENDING);
      drive.setWantedState(DriveState.DEFAULT);
    } else {
      continueClimbing = true;
      drive.setWantedState(DriveState.DEFAULT);
    }

    if (climber.getTimesTriggered() && continueClimbing && !(climber.getPosition() > -105)) {
      climber.setWantedState(ClimbState.RETRACTING);
      drive.setWantedState(DriveState.AUTO_CLIMB);
    } else if (continueClimbing) {
      climber.setWantedState(ClimbState.IDLE);
      drive.setWantedState(DriveState.DEFAULT);
    }

  }

  public void handleLollipopPickup() {
    lights.setWantedState(LightsState.INTAKING);
    elevator.setWantedState(ElevatorState.LOLLIPOP);
    manipulator.setWantedState(ManipulatorState.CORAL_INTAKE);
    pivot.setWantedState(PivotState.LOLLIPOP);
    if (pivot.getPivotPosition() > 15.0 / 360.0) {
      twist.setWantedState(TwistState.SIDE);
    }
  }

  @Override
  public void periodic() {
    pivot.updateIntakeItem(manipulator.getArmItem());
    twist.updateIntakeItem(manipulator.getArmItem());
    if (OI.getDriverA()) {
      isClimbing = false;
    }
    if (OI.driverY.getAsBoolean()) {
      isClimbing = true;
    }
    currentSuperState = handleStateTransitions();
    if (currentSuperState != SuperState.ZERO) {
      firstTimeZero = true;
    }
    if (currentSuperState != SuperState.DEFAULT) {
      firstTimeDefault = true;
    }
    if (currentSuperState == SuperState.AUTO_L1_PLACE || currentSuperState == SuperState.AUTO_SCORE_L1
        || currentSuperState == SuperState.L1_PLACE) {
      manipulator.inL1State = true;
    } else {
      manipulator.inL1State = false;
    }
    Logger.recordOutput("Super State", currentSuperState);
    if (currentSuperState != SuperState.OUTAKE_IDLE) {
      outakeIdleInit = false;
    }
    if (currentSuperState != SuperState.AUTO_SCORE_L4) {
      hasPlaced = false;
    }
    if (currentSuperState != SuperState.AUTO_CLIMB) {
      continueClimbing = false;
      climber.timesTriggered = 0;
    }
    // Logger.recordOutput("Hit Time", hitAutoSetpointTime);
    applyStates();
  }
}
