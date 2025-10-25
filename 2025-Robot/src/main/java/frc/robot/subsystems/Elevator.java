package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Constants.SetPoints.ElevatorPosition;
import frc.robot.subsystems.Manipulator.ArmItem;

public class Elevator extends SubsystemBase {
  private final TalonFX elevatorMotorMaster = new TalonFX(Constants.CANInfo.MASTER_ELEVATOR_MOTOR_ID,
      new CANBus(Constants.CANInfo.CANBUS_NAME));
  private final TalonFX elevatorMotorFollower = new TalonFX(Constants.CANInfo.FOLLOWER_ELEVATOR_MOTOR_ID,
      new CANBus(Constants.CANInfo.CANBUS_NAME));

  private final TorqueCurrentFOC torqueCurrentFOCRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);
  private final double elevatorAcceleration = 1482542976.0;
  private final double elevatorCruiseVelocity = 449929104911.0;
  private final MotionMagicTorqueCurrentFOC elevatorMotionProfileRequest = new MotionMagicTorqueCurrentFOC(0);

  public enum ElevatorState {
    DEFAULT,
  }

  private double idleTime;
  private boolean firstTimeIdle = true;
  private ElevatorState wantedState = ElevatorState.DEFAULT;
  private ElevatorState systemState = ElevatorState.DEFAULT;
  private double distanceFromL23DriveSetpoint = 0.0;
  private boolean firstTimeDefault = false;

  private ArmItem intakeItem = ArmItem.NONE;

  public void updateIntakeItem(ArmItem intakeItem) {
    this.intakeItem = intakeItem;
  }

  public void updateDistanceFromL23DriveSetpoint(double distanceFromL23DriveSetpoint) {
    this.distanceFromL23DriveSetpoint = distanceFromL23DriveSetpoint;
  }

  public double getElevatorL3ScoreSetpoint() {
    if (ElevatorPosition.kAUTOL3.meters - Math.tan(Math.PI / 6) * distanceFromL23DriveSetpoint * 1.2 > 0.0) {
      return (ElevatorPosition.kAUTOL3.meters
          - 3.5 / 39.37 - Math.tan(Math.PI / 6) * distanceFromL23DriveSetpoint * 1.5);
    } else {
      return 0.0;
    }
  }

  public Elevator() {

  }

  public void teleopInit() {
    firstTimeIdle = true;
    firstTimeDefault = false;
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    currentLimitsConfigs.StatorCurrentLimitEnable = true;
    currentLimitsConfigs.SupplyCurrentLimitEnable = true;
    currentLimitsConfigs.StatorCurrentLimit = 60;
    currentLimitsConfigs.SupplyCurrentLimit = 60;
    elevatorMotorMaster.getConfigurator().apply(currentLimitsConfigs);
    elevatorMotorFollower.getConfigurator().apply(currentLimitsConfigs);
  }

  public void autoInit() {
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    currentLimitsConfigs.StatorCurrentLimitEnable = true;
    currentLimitsConfigs.SupplyCurrentLimitEnable = true;
    currentLimitsConfigs.StatorCurrentLimit = 60;
    currentLimitsConfigs.SupplyCurrentLimit = 60;
    elevatorMotorMaster.getConfigurator().apply(currentLimitsConfigs);
    elevatorMotorFollower.getConfigurator().apply(currentLimitsConfigs);
  }

  public void init() {
    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    double elevatorMultiplier = 45.01 / 33.39;
    elevatorConfig.Slot0.kP = 33.39 * elevatorMultiplier;
    elevatorConfig.Slot0.kI = 0.0 * elevatorMultiplier;
    elevatorConfig.Slot0.kD = 2.7 * elevatorMultiplier;
    elevatorConfig.Slot0.kG = 4.499 * elevatorMultiplier;
    elevatorConfig.Slot1.kP = 75.83 * elevatorMultiplier;
    elevatorConfig.Slot1.kI = 0.0 * elevatorMultiplier;
    elevatorConfig.Slot1.kD = 4.690 * elevatorMultiplier;
    elevatorConfig.Slot1.kG = 8.044 * elevatorMultiplier;
    elevatorConfig.Slot2.kP = 33.39 * elevatorMultiplier * 0.5;
    elevatorConfig.Slot2.kI = 0.0 * elevatorMultiplier * 0.5;
    elevatorConfig.Slot2.kD = 2.7 * elevatorMultiplier * 0.5;
    elevatorConfig.Slot2.kG = 4.499 * elevatorMultiplier * 0.5;
    elevatorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    elevatorConfig.Slot1.GravityType = GravityTypeValue.Elevator_Static;
    elevatorConfig.Slot2.GravityType = GravityTypeValue.Elevator_Static;
    elevatorConfig.MotionMagic.MotionMagicAcceleration = this.elevatorAcceleration;
    elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = this.elevatorCruiseVelocity;
    elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.StatorCurrentLimit = 60;
    elevatorConfig.CurrentLimits.SupplyCurrentLimit = 60;

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

  public void moveElevatorToPosition(double position) {
    if (position < Constants.Ratios.ELEVATOR_FIRST_STAGE) {
      elevatorMotorMaster.setControl(
          elevatorMotionProfileRequest.withPosition(Constants.Ratios.elevatorMetersToRotations(position)).withSlot(0));
      elevatorMotorFollower.setControl(
          elevatorMotionProfileRequest.withPosition(-Constants.Ratios.elevatorMetersToRotations(position)).withSlot(0));
    } else {
      if (position > Constants.inchesToMeters(64.0) && getElevatorPosition() > Constants.inchesToMeters(62.0)) {
        moveWithTorque(18, 0.20);
        // System.out.println("running torque");
      } else {
        elevatorMotorMaster.setControl(
            elevatorMotionProfileRequest.withPosition(Constants.Ratios.elevatorMetersToRotations(position))
                .withSlot(1));
        elevatorMotorFollower.setControl(
            elevatorMotionProfileRequest.withPosition(-Constants.Ratios.elevatorMetersToRotations(position))
                .withSlot(1));
      }
    }
  }

  public void moveElevatorToPositionSlow(double position) {
    if (position < Constants.Ratios.ELEVATOR_FIRST_STAGE) {
      elevatorMotorMaster.setControl(
          elevatorMotionProfileRequest.withPosition(Constants.Ratios.elevatorMetersToRotations(position)).withSlot(2));
      elevatorMotorFollower.setControl(
          elevatorMotionProfileRequest.withPosition(-Constants.Ratios.elevatorMetersToRotations(position)).withSlot(2));
    } else {
      if (position > Constants.inchesToMeters(64.0) && getElevatorPosition() > Constants.inchesToMeters(62.0)) {
        moveWithTorque(18, 0.20);
        // System.out.println("running torque");
      } else {
        elevatorMotorMaster.setControl(
            elevatorMotionProfileRequest.withPosition(Constants.Ratios.elevatorMetersToRotations(position))
                .withSlot(1));
        elevatorMotorFollower.setControl(
            elevatorMotionProfileRequest.withPosition(-Constants.Ratios.elevatorMetersToRotations(position))
                .withSlot(1));
      }
    }
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
      default:
        return ElevatorState.DEFAULT;
    }
  }

  public boolean getZeroed() {
    if (Math.abs(elevatorMotorMaster.getStatorCurrent().getValueAsDouble()) > 10.0
        && Math.abs(elevatorMotorMaster.getVelocity().getValueAsDouble()) < 5.0) {
      return true;
    } else {
      return false;
    }
  }

  private double zeroTime = 0.0;

  @Override
  public void periodic() {
    systemState = handleStateTransition();
    if (systemState != ElevatorState.DEFAULT || OI.driverMenuButton.getAsBoolean()) {
      firstTimeDefault = false;
      idleTime = Timer.getFPGATimestamp();
      zeroTime = 0.0;
    }
    // System.out.println("Elevator Current: " +
    // elevatorMotorMaster.getStatorCurrent().getValueAsDouble());
    // Logger.recordOutput("Elevator Current",
    // elevatorMotorMaster.getStatorCurrent().getValueAsDouble());
    // Logger.recordOutput("Elevator Idle Time", idleTime);
    // Logger.recordOutput("First Time Idle", firstTimeIdle);
    Logger.recordOutput("Elevator State", systemState);
    // Logger.recordOutput("Elevator Velocity",
    // Constants.Ratios.elevatorRotationsToMeters(elevatorMotorMaster.getVelocity().getValueAsDouble()));
    Logger.recordOutput("Elevator Height", getElevatorPosition() * 39.37);
    switch (systemState) {
      default:
        moveWithPercent(0.0);
        break;
    }
  }
}
