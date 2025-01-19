package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private final TalonFX elevatorMotorMaster = new TalonFX(Constants.CANInfo.MASTER_ELEVATOR_MOTOR_ID,
      new CANBus(Constants.CANInfo.CANBUS_NAME));
  private final TalonFX elevatorMotorFollower = new TalonFX(Constants.CANInfo.FOLLOWER_ELEVATOR_MOTOR_ID,
      new CANBus(Constants.CANInfo.CANBUS_NAME));

  private final PositionTorqueCurrentFOC positionTorqueFOCRequest = new PositionTorqueCurrentFOC(0);
  private final TorqueCurrentFOC torqueCurrentFOCRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);

  public enum ElevatorState {
    IDLE,
    UP,
    MID,
    DOWN,
    OFF,
    L3,
    L2,
    ALGAE,
  }

  private double idleTime;
  private boolean firstTimeIdle = true;
  private ElevatorState wantedState = ElevatorState.IDLE;
  private ElevatorState systemState = ElevatorState.IDLE;

  public Elevator() {
  }

  public void teleopInit() {
    firstTimeIdle = true;
  }

  public void init() {
    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    elevatorConfig.Slot0.kP = 3.5;
    elevatorConfig.Slot0.kI = 0.0;
    elevatorConfig.Slot0.kD = 0.4;
    elevatorConfig.Slot0.kG = 0.5;
    elevatorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.StatorCurrentLimit = 60;
    elevatorConfig.CurrentLimits.SupplyCurrentLimit = 60;
    // elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =

    elevatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    elevatorMotorMaster.getConfigurator().apply(elevatorConfig);
    elevatorMotorFollower.getConfigurator().apply(elevatorConfig);
    elevatorMotorMaster.setNeutralMode(NeutralModeValue.Brake);
    elevatorMotorFollower.setNeutralMode(NeutralModeValue.Brake);
    elevatorMotorMaster.setPosition(0.0);
    elevatorMotorFollower.setPosition(0.0);
  }

  public void moveWithPercent(double percent) {
    elevatorMotorMaster.set(percent);
    elevatorMotorFollower.set(-percent);
  }

  public void moveWithTorque(double current, double maxPercent) {
    elevatorMotorMaster.setControl(torqueCurrentFOCRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
    elevatorMotorFollower.setControl(torqueCurrentFOCRequest.withOutput(-current).withMaxAbsDutyCycle(maxPercent));
  }

  public void moveElevatorToPosition(Constants.SetPoints.ElevatorPosition position) {
    elevatorMotorMaster
        .setControl(positionTorqueFOCRequest.withPosition(Constants.Ratios.elevatorMetersToRotations(position.meters)));
    elevatorMotorFollower
        .setControl(
            positionTorqueFOCRequest.withPosition(-Constants.Ratios.elevatorMetersToRotations(position.meters)));
  }

  public double getElevatorPosition() {
    return Constants.Ratios.elevatorRotationsToMeters(elevatorMotorMaster.getPosition().getValueAsDouble());
  }

  public void setElevatorEncoderPosition(double position) {
    elevatorMotorMaster.setPosition(position);
    elevatorMotorFollower.setPosition(position);
  }

  public void setWantedState(ElevatorState wantedState) {
    this.wantedState = wantedState;
  }

  private ElevatorState handleStateTransition() {
    switch (wantedState) {
      case IDLE:
        return ElevatorState.IDLE;
      case UP:
        return ElevatorState.UP;
      case MID:
        return ElevatorState.MID;
      case DOWN:
        return ElevatorState.DOWN;
      case L3:
        return ElevatorState.L3;
      case L2:
        return ElevatorState.L2;
      case ALGAE:
        return ElevatorState.ALGAE;
      default:
        return ElevatorState.OFF;
    }
  }

  @Override
  public void periodic() {
    systemState = handleStateTransition();

    Logger.recordOutput("Elevator State", systemState);
    Logger.recordOutput("1 position", elevatorMotorMaster.getPosition().getValueAsDouble());
    Logger.recordOutput("2 position", elevatorMotorFollower.getPosition().getValueAsDouble());
    switch (systemState) {
      case UP:
        firstTimeIdle = true;
        moveElevatorToPosition(Constants.SetPoints.ElevatorPosition.kUP);
        break;
      case MID:
        firstTimeIdle = true;
        moveElevatorToPosition(Constants.SetPoints.ElevatorPosition.kMID);
        break;
      case DOWN:
        firstTimeIdle = true;
        moveElevatorToPosition(Constants.SetPoints.ElevatorPosition.kDOWN);
        break;
      case L3:
        firstTimeIdle = true;
        moveElevatorToPosition(Constants.SetPoints.ElevatorPosition.kL3);
        break;
      case L2:
        firstTimeIdle = true;
        moveElevatorToPosition(Constants.SetPoints.ElevatorPosition.kL2);
        break;
      case ALGAE:
        firstTimeIdle = true;
        moveElevatorToPosition(Constants.SetPoints.ElevatorPosition.kALGAE);
        break;
      case IDLE:

        if (firstTimeIdle) {
          idleTime = Timer.getFPGATimestamp();
          firstTimeIdle = false;
        }
        if (Math
            .abs(Constants.Ratios.elevatorRotationsToMeters(elevatorMotorMaster.getVelocity().getValueAsDouble())) < 0.1
            && Timer.getFPGATimestamp() - idleTime > 0.1) {
          moveWithPercent(0.0);
          setElevatorEncoderPosition(0.0);
        } else {
          moveWithTorque(-25, 0.6);
        }
        Logger.recordOutput("Elevator Current", elevatorMotorMaster.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Elevator MPS",
            Constants.Ratios.elevatorRotationsToMeters(elevatorMotorMaster.getVelocity().getValueAsDouble()));
        break;
      default:
        firstTimeIdle = true;
        moveWithPercent(0.0);
    }
  }
}
