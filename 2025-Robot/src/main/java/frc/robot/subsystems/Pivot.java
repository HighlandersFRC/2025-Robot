package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {

  private final TalonFX pivotMotor = new TalonFX(Constants.CANInfo.PIVOT_MOTOR_ID,
      new CANBus(Constants.CANInfo.CANBUS_NAME));
  private final CANcoder pivotCANcoder = new CANcoder(Constants.CANInfo.PIVOT_CANCODER_ID,
      new CANBus(Constants.CANInfo.CANBUS_NAME));

  private final double pivotJerk = 0.0;
  private final double pivotAcceleration = 4.0 * Constants.Ratios.PIVOT_GEAR_RATIO;
  private final double pivotCruiseVelocity = 4.0 * Constants.Ratios.PIVOT_GEAR_RATIO;

  private final double pivotProfileScalarFactor = 1;

  private final DynamicMotionMagicVoltage pivotMotionProfileRequest = new DynamicMotionMagicVoltage(0,
      pivotCruiseVelocity,
      pivotAcceleration,
      pivotJerk);

  public Pivot() {
  }

  public void init() {
    pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    pivotConfig.Slot0.kP = 50.0 * Constants.Ratios.PIVOT_GEAR_RATIO;
    pivotConfig.Slot0.kI = 0.0;
    pivotConfig.Slot0.kD = 10.0 * Constants.Ratios.PIVOT_GEAR_RATIO;
    pivotConfig.MotionMagic.MotionMagicJerk = this.pivotJerk;
    pivotConfig.MotionMagic.MotionMagicAcceleration = this.pivotAcceleration;
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = this.pivotCruiseVelocity;
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.StatorCurrentLimit = 40;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = 40;
    // pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    // pivotConfig.Feedback.FeedbackRemoteSensorID = pivotCANcoder.getDeviceID();
    // pivotConfig.Feedback.SensorToMechanismRatio = 1.0;
    // pivotConfig.Feedback.RotorToSensorRatio = Constants.Ratios.PIVOT_GEAR_RATIO;

    pivotMotor.getConfigurator().apply(pivotConfig);
    pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    pivotMotor.setPosition(0.0);
  }

  public void pivotToPosition(double pivotPosition) {
    Logger.recordOutput("Pivot Setpoint", (pivotPosition));
    pivotMotor.setControl(this.pivotMotionProfileRequest
        .withPosition(pivotPosition * Constants.Ratios.PIVOT_GEAR_RATIO)
        .withAcceleration(this.pivotAcceleration * pivotProfileScalarFactor)
        .withJerk(
            this.pivotJerk * pivotProfileScalarFactor));
  }

  public double getPivotPosition() {
    return (pivotMotor.getPosition().getValueAsDouble() / Constants.Ratios.PIVOT_GEAR_RATIO);
  }

  public void setpivotEncoderPosition(double position) {
    pivotMotor.setPosition(position);
  }

  public void setPivotPercent(double percent) {
    pivotMotor.set(percent);
  }

  public enum PivotFlip {
    FRONT,
    BACK,
  }

  public enum PivotState {
    AUTO_L1,
    AUTO_L23,
    AUTO_L4,
    L1,
    L23,
    L4,
    PROCESSOR,
    NET,
    // FEEDER_FRONT,
    // FEEDER_BACK,
    FEEDER,
    GROUND_CORAL_FRONT,
    GROUND_CORAL_PREP_BACK,
    GROUND_CORAL_BACK,
    GROUND_ALGAE,
    REEF_ALGAE,
    DEFAULT,
    SCORE_L1,
    SCORE_L23,
    SCORE_L4,
    AUTO_SCORE_L1,
    AUTO_SCORE_L23,
    AUTO_SCORE_L4,
  }

  private PivotState wantedState = PivotState.DEFAULT;
  private PivotState systemState = PivotState.DEFAULT;

  private PivotFlip wantedFlip = PivotFlip.FRONT;
  private PivotFlip systemFlip = PivotFlip.FRONT;

  public void setWantedState(PivotState wantedState) {
    this.wantedState = wantedState;
  }

  public void setWantedFlip(PivotFlip wantedFlip) {
    this.wantedFlip = wantedFlip;
  }

  private PivotFlip handleFlipTransition() {
    switch (wantedFlip) {
      case FRONT:
        return PivotFlip.FRONT;
      case BACK:
        return PivotFlip.BACK;
      default:
        return PivotFlip.FRONT;
    }
  }

  private PivotState handleStateTransition() {
    switch (wantedState) {
      case DEFAULT:
        return PivotState.DEFAULT;
      case L1:
        return PivotState.L1;
      case L23:
        return PivotState.L23;
      case L4:
        return PivotState.L4;
      case AUTO_L1:
        return PivotState.AUTO_L1;
      case AUTO_L23:
        return PivotState.AUTO_L23;
      case AUTO_L4:
        return PivotState.AUTO_L4;
      // case FEEDER_FRONT:
      //   return PivotState.FEEDER_FRONT;
      // case FEEDER_BACK:
      //   return PivotState.FEEDER_BACK;
      case FEEDER:
        return PivotState.FEEDER;
      case REEF_ALGAE:
        return PivotState.REEF_ALGAE;
      case GROUND_CORAL_FRONT:
        return PivotState.GROUND_CORAL_FRONT;
      case GROUND_CORAL_BACK:
        return PivotState.GROUND_CORAL_BACK;
      case GROUND_CORAL_PREP_BACK:
        return PivotState.GROUND_CORAL_PREP_BACK;
      case GROUND_ALGAE:
        return PivotState.GROUND_ALGAE;
      case PROCESSOR:
        return PivotState.PROCESSOR;
      case NET:
        return PivotState.NET;
      case SCORE_L1:
        return PivotState.SCORE_L1;
      case SCORE_L23:
        return PivotState.SCORE_L23;
      case SCORE_L4:
        return PivotState.SCORE_L4;
      case AUTO_SCORE_L1:
        return PivotState.AUTO_SCORE_L1;
      case AUTO_SCORE_L23:
        return PivotState.AUTO_SCORE_L23;
      case AUTO_SCORE_L4:
        return PivotState.AUTO_SCORE_L4;
      default:
        return PivotState.DEFAULT;
    }
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Pivot Position", getPivotPosition());

    // Logger.recordOutput("Pivot Output",
    // pivotMotor.getClosedLoopOutput().getValueAsDouble());
    // Logger.recordOutput("Pivot Current",
    // pivotMotor.getStatorCurrent().getValueAsDouble());
    systemState = handleStateTransition();
    systemFlip = handleFlipTransition();
    Logger.recordOutput("Pivot State", systemState);
    // switch (systemState) {
    //   case DEFAULT:
    //     pivotToPosition(Constants.SetPoints.PivotPosition.kDEFAULT.rotations);
    //     break;
    //   case GROUND_ALGAE:
    //     pivotToPosition(Constants.SetPoints.PivotPosition.kGROUNDALGAE.rotations);
    //     break;
    //   case GROUND_CORAL_FRONT:
    //     pivotToPosition(Constants.SetPoints.PivotPosition.kGROUNDCORALFRONT.rotations);
    //     break;
    //   case GROUND_CORAL_BACK:
    //     pivotToPosition(Constants.SetPoints.PivotPosition.kGROUNDCORALBACK.rotations);
    //     break;
    //   case GROUND_CORAL_PREP_BACK:
    //     pivotToPosition(Constants.SetPoints.PivotPosition.kGROUNDCORALPREPBACK.rotations);
    //     break;
    //   case L1:
    //     pivotToPosition(Constants.SetPoints.PivotPosition.kL1.rotations);
    //     break;
    //   case SCORE_L1:
    //     break;
    //   case L23:
    //     pivotToPosition(Constants.SetPoints.PivotPosition.kL23.rotations);
    //     break;
    //   case SCORE_L23:
    //     pivotToPosition(0.25);
    //     break;
    //   case L4:
    //     pivotToPosition(Constants.SetPoints.PivotPosition.kL4.rotations);
    //     break;
    //   case SCORE_L4:
    //     pivotToPosition(0.25);
    //     break;
    //   case AUTO_SCORE_L23:
    //     switch (systemFlip) {
    //       case FRONT:
    //         pivotToPosition(Constants.SetPoints.PivotPosition.kAUTOL23SCORE.rotations);
    //         break;
    //       case BACK:
    //         pivotToPosition(-Constants.SetPoints.PivotPosition.kAUTOL23SCORE.rotations);
    //         break;
    //       default:
    //         pivotToPosition(Constants.SetPoints.PivotPosition.kAUTOL23SCORE.rotations);
    //         break;
    //     }
    //     break;
    //   case AUTO_SCORE_L4:
    //     switch (systemFlip) {
    //       case FRONT:
    //         pivotToPosition(Constants.SetPoints.PivotPosition.kAUTOL4SCORE.rotations);
    //         break;
    //       case BACK:
    //         pivotToPosition(-Constants.SetPoints.PivotPosition.kAUTOL4SCORE.rotations);
    //         break;
    //       default:
    //         pivotToPosition(Constants.SetPoints.PivotPosition.kAUTOL4SCORE.rotations);
    //         break;
    //     }
    //     break;
    //   // case FEEDER_FRONT:
    //   //   pivotToPosition(Constants.SetPoints.PivotPosition.kFEEDER.rotations);
    //   //   break;
    //   // case FEEDER_BACK:
    //   //   pivotToPosition(-Constants.SetPoints.PivotPosition.kFEEDER.rotations);
    //   //   break;
    //   case FEEDER:
    //     switch (systemFlip) {
    //       case FRONT:
    //         pivotToPosition(Constants.SetPoints.PivotPosition.kFEEDER.rotations);
    //         break;
    //       case BACK:
    //         pivotToPosition(-Constants.SetPoints.PivotPosition.kFEEDER.rotations);
    //         break;
    //       default:
    //         pivotToPosition(Constants.SetPoints.PivotPosition.kFEEDER.rotations);
    //         break;
    //     }
    //     break;
    //   case AUTO_L1:
    //     break;
    //   case AUTO_L23:
    //     switch (systemFlip) {
    //       case FRONT:
    //         pivotToPosition(Constants.SetPoints.PivotPosition.kAUTOL23.rotations);
    //         break;
    //       case BACK:
    //         pivotToPosition(-Constants.SetPoints.PivotPosition.kAUTOL23.rotations);
    //         break;
    //       default:
    //         pivotToPosition(Constants.SetPoints.PivotPosition.kAUTOL23.rotations);
    //         break;
    //     }
    //     break;
    //   case AUTO_L4:
    //     switch (systemFlip) {
    //       case FRONT:
    //         pivotToPosition(Constants.SetPoints.PivotPosition.kAUTOL4.rotations);
    //         break;
    //       case BACK:
    //         pivotToPosition(-Constants.SetPoints.PivotPosition.kAUTOL4.rotations);
    //         break;
    //       default:
    //         pivotToPosition(Constants.SetPoints.PivotPosition.kAUTOL4.rotations);
    //         break;
    //     }
    //     break;
    //   default:
    //     setPivotPercent(0.0);
    //     break;
    // }
    // This method will be called once per scheduler run
  }
}
