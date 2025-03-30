package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.IntakeItem;

public class Twist extends SubsystemBase {

  private final TalonFX twistMotor = new TalonFX(Constants.CANInfo.TWIST_MOTOR_ID,
      new CANBus(Constants.CANInfo.CANBUS_NAME));
  private final CANcoder twistCANcoder = new CANcoder(Constants.CANInfo.TWIST_CANCODER_ID,
      new CANBus(Constants.CANInfo.CANBUS_NAME));

  private final double twistJerk = 20.0;
  private final double twistAcceleration = 50.0;
  private final double twistCruiseVelocity = 200.0;

  // private double zeroInitTime = 0.0;

  private final MotionMagicExpoVoltage twistTorqueCurrentFOC = new MotionMagicExpoVoltage(
      0.0);
  private final TorqueCurrentFOC torqueCurrentFOCRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);

  public Twist() {
  }

  public void init() {
    TalonFXConfiguration twistConfig = new TalonFXConfiguration();
    twistConfig.Slot0.kP = 50.0;
    twistConfig.Slot0.kI = 0.0;
    twistConfig.Slot0.kD = 4.6;
    twistConfig.Slot0.kS = 5.0;
    twistConfig.Slot1.kP = 13.0;
    twistConfig.Slot1.kI = 0.0;
    twistConfig.Slot1.kD = 5.0;
    twistConfig.Slot1.kS = 1.0;
    twistConfig.MotionMagic.MotionMagicJerk = this.twistJerk;
    twistConfig.MotionMagic.MotionMagicAcceleration = this.twistAcceleration;
    twistConfig.MotionMagic.MotionMagicCruiseVelocity = this.twistCruiseVelocity;
    twistConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    twistConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    twistConfig.CurrentLimits.StatorCurrentLimit = 60;
    twistConfig.CurrentLimits.SupplyCurrentLimit = 60;
    twistConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    twistConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    twistConfig.Feedback.FeedbackRemoteSensorID = twistCANcoder.getDeviceID();
    twistConfig.Feedback.SensorToMechanismRatio = 1.0;
    twistConfig.Feedback.RotorToSensorRatio = Constants.Ratios.TWIST_GEAR_RATIO;
    twistTorqueCurrentFOC.EnableFOC = true;
    twistMotor.getConfigurator().apply(twistConfig);
    twistMotor.setNeutralMode(NeutralModeValue.Brake);
    twistMotor.setPosition(0.0);
  }

  public void teleopInit() {
    // zeroInitTime = Timer.getFPGATimestamp();
  }

  private IntakeItem intakeItem = IntakeItem.NONE;

  public void updateIntakeItem(IntakeItem intakeItem) {
    this.intakeItem = intakeItem;
  }

  public void twistToPosition(double rotations) {

    if (intakeItem == IntakeItem.ALGAE) { // TODO: UNCOMMENT IF YOUR WANT THE TWIST TO MOVE
      twistMotor.setControl(this.twistTorqueCurrentFOC // TODO: UNCOMMENT IF YOUR WANT THE TWIST TO MOVE
          .withPosition(rotations).withEnableFOC(true).withSlot(1)); // TODO: UNCOMMENT IF YOUR WANT THE TWIST TO MOVE
    } else { // TODO: UNCOMMENT IF YOUR WANT THE TWIST TO MOVE
      twistMotor.setControl(this.twistTorqueCurrentFOC // TODO: UNCOMMENT IF YOUR WANT THE TWIST TO MOVE
          .withPosition(rotations).withEnableFOC(true).withSlot(0)); // TODO: UNCOMMENT IF YOUR WANT THE TWIST TO MOVE
    } // TODO: UNCOMMENT IF YOUR WANT THE TWIST TO MOVE

    // Logger.recordOutput("Twist Setpoint", rotations);
    // twistMotor.setControl(this.twistTorqueCurrentFOC
    // .withPosition(rotations).withEnableFOC(true));
  }

  public void setTwistPercent(double percent) {
    twistMotor.set(percent);
  }

  public void setTwistTorque(double torque, double maxPercent) {
    twistMotor.setControl(torqueCurrentFOCRequest.withOutput(torque).withMaxAbsDutyCycle(maxPercent));
  }

  public double getTwistPosition() {
    return 360 * twistMotor.getPosition().getValueAsDouble();
  }

  public void setTwistEncoderPosition(double position) {
    twistMotor.setPosition(position);
  }

  public enum TwistState {
    UP,
    SIDE,
    DOWN,
  }

  private TwistState wantedState = TwistState.SIDE;
  private TwistState systemState = TwistState.SIDE;

  public void setWantedState(TwistState wantedState) {
    this.wantedState = wantedState;
  }

  private TwistState handleStateTransition() {
    switch (wantedState) {
      case UP:
        return TwistState.UP;
      case SIDE:
        return TwistState.SIDE;
      case DOWN:
        return TwistState.DOWN;
      default:
        return TwistState.UP;
    }
  }

  @Override
  public void periodic() {
    // System.out.println("Position" + getTwistPosition());
    systemState = handleStateTransition();
    // System.out.println("Twist Current: " +
    // twistMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Twist State: ", systemState);
    Logger.recordOutput("Twist Position", getTwistPosition());
    // System.out.println("Twist Position: " + getTwistPosition());
    // System.out.println("Twist State: " + systemState);
    // Logger.recordOutput("Twist Error",
    // twistMotor.getClosedLoopError().getValueAsDouble());
    // Logger.recordOutput("Twist Current",
    // twistMotor.getStatorCurrent().getValueAsDouble());
    // Logger.recordOutput("Twist MPS",
    // (twistMotor.getVelocity().getValueAsDouble()));
    switch (systemState) {
      case DOWN:
        // if (!startedZero) {
        // }
        // if (Timer.getFPGATimestamp() - zeroInitTime > 1.3) {
        // setTwistPercent(0.0);
        // setTwistEncoderPosition(0.0);
        // } else {
        // setTwistTorque(10, 0.3);
        // }
        // startedZero = false;
        // zeroInitTime = 0.0;
        twistToPosition(0.25);
        break;
      case SIDE:
        // zeroInitTime = 0.0;
        twistToPosition(0.0);
        break;
      case UP:
        // zeroInitTime = 0.0;
        twistToPosition(-0.25);
        break;
      default:
        // zeroInitTime = 0.0;
        twistToPosition(0.0);
        break;
    }
  }
}