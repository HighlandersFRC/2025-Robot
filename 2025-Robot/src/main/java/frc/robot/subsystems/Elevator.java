package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SetPoints.ElevatorPosition;

public class Elevator extends SubsystemBase {
  private final TalonFX elevatorMotorMaster = new TalonFX(Constants.CANInfo.MASTER_ELEVATOR_MOTOR_ID,
      new CANBus(Constants.CANInfo.CANBUS_NAME));
  private final TalonFX elevatorMotorFollower = new TalonFX(Constants.CANInfo.FOLLOWER_ELEVATOR_MOTOR_ID,
      new CANBus(Constants.CANInfo.CANBUS_NAME));

  private final PositionTorqueCurrentFOC positionTorqueFOCRequest = new PositionTorqueCurrentFOC(0).withFeedForward(0).withVelocity(0);
  private final TorqueCurrentFOC torqueCurrentFOCRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);
  private final double elevatorAcceleration = 100.0;
  private final double elevatorCruiseVelocity = 50.0;
  private final double elevatorFeedForward = 10.0;
  private final double elevatorProfileScalarFactor = 1;

  private final DynamicMotionMagicTorqueCurrentFOC elevatorMotionProfileRequest = new DynamicMotionMagicTorqueCurrentFOC(0,
      elevatorCruiseVelocity,
      elevatorAcceleration,
      0.0);
  public enum ElevatorState {
    DEFAULT,
    L1,
    L2,
    L3,
    L4,
    FEEDER_INTAKE,
    L2_ALGAE,
    L3_ALGAE,
    GROUND_INTAKE,
    PROCESSOR,
    SCORE,
    NET,

  }

  private double idleTime;
  private boolean firstTimeIdle = true;
  private ElevatorState wantedState = ElevatorState.DEFAULT;
  private ElevatorState systemState = ElevatorState.DEFAULT;

  public Elevator() {
  }

  public void teleopInit() {
    firstTimeIdle = true;
  }

  public void init() {
    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    elevatorConfig.Slot0.kP = 30.0;
    elevatorConfig.Slot0.kI = 0.0;
    elevatorConfig.Slot0.kD = 0.0;
    elevatorConfig.Slot0.kG = 13.0;
    elevatorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    elevatorConfig.MotionMagic.MotionMagicAcceleration = this.elevatorAcceleration;
    elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = this.elevatorCruiseVelocity;
    elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.StatorCurrentLimit = 60;
    elevatorConfig.CurrentLimits.SupplyCurrentLimit = 60;
    // elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =

    elevatorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

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
    // elevatorMotorMaster
    //     .setControl(positionTorqueFOCRequest.withPosition(Constants.Ratios.elevatorMetersToRotations(position.meters)));
    // elevatorMotorFollower
    //     .setControl(
    //         positionTorqueFOCRequest.withPosition(-Constants.Ratios.elevatorMetersToRotations(position.meters)));
    elevatorMotorMaster.setControl(elevatorMotionProfileRequest.withPosition(Constants.Ratios.elevatorMetersToRotations(position.meters)).withVelocity(elevatorCruiseVelocity*elevatorProfileScalarFactor).withAcceleration(elevatorAcceleration*elevatorProfileScalarFactor));
    elevatorMotorFollower.setControl(elevatorMotionProfileRequest.withPosition(-Constants.Ratios.elevatorMetersToRotations(position.meters)).withVelocity(elevatorCruiseVelocity*elevatorProfileScalarFactor).withAcceleration(elevatorAcceleration*elevatorProfileScalarFactor));
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
      case DEFAULT:
        return ElevatorState.DEFAULT;
      case L1:
        return ElevatorState.L1;
      case L2:
        return ElevatorState.L2;
      case L3:
        return ElevatorState.L3;
      case L4:
        return ElevatorState.L4;
      case FEEDER_INTAKE:
        return ElevatorState.FEEDER_INTAKE;
      case L2_ALGAE:
        return ElevatorState.L2_ALGAE;
      case L3_ALGAE:
        return ElevatorState.L3_ALGAE;
      case GROUND_INTAKE:
        return ElevatorState.GROUND_INTAKE;
      case PROCESSOR:
        return ElevatorState.PROCESSOR;
      case SCORE:
        return ElevatorState.SCORE;
      case NET:
        return ElevatorState.NET;
      default:
        return ElevatorState.DEFAULT;
    }
  }

  @Override
  public void periodic() {
    systemState = handleStateTransition();

    Logger.recordOutput("Elevator State", systemState);
    Logger.recordOutput("Elevator Height", getElevatorPosition()*39.37);
    switch (systemState) {
      case GROUND_INTAKE:
        firstTimeIdle = true;
        moveElevatorToPosition(ElevatorPosition.kGROUNDPICKUP);
        break;
      case L2:
        firstTimeIdle = true;
        moveElevatorToPosition(ElevatorPosition.kL2);
        break;
      default:
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
          moveWithTorque(-20, 0.4);
        }
        break;
    }
  }
}
