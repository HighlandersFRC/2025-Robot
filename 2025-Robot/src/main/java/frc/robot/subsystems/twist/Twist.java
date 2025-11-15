package frc.robot.subsystems.twist;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.manipulator.Manipulator.ArmItem;

public class Twist extends SubsystemBase {
  TwistIO io;

  public boolean algaeMode = false;

  public Twist() {
    if (RobotBase.isReal()) {
      io = new TwistIOComp();
    } else {
      io = new TwistIOSim();
    }
  }

  public void init() {
    io.init();
  }

  public void teleopInit() {
  }

  private ArmItem _armItem = ArmItem.NONE;

  public void updateIntakeItem(ArmItem armItem) {
    this._armItem = armItem;
  }

  public void twistToPosition(double rotations) {
    io.setPosition(rotations, _armItem);
  }

  public void setTwistPercent(double percent) {
    io.setPercent(percent);
  }

  public void setTwistTorque(double torque, double maxPercent) {
    io.setTorque(torque, maxPercent);
  }

  public double getTwistPosition() {
    return io.getPosition();
  }

  public void setTwistEncoderPosition(double position) {
    io.setEncoderPosition(position);
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
    io.updateInputs(systemState);
    systemState = handleStateTransition();
    switch (systemState) {
      case DOWN:
        twistToPosition(Constants.SetPoints.TwistSetpoints.TWIST_DOWN);
        break;
      case SIDE:
        twistToPosition(Constants.SetPoints.TwistSetpoints.TWIST_SIDE);
        break;
      case UP:
        twistToPosition(Constants.SetPoints.TwistSetpoints.TWIST_UP);
        break;
      default:
        twistToPosition(Constants.SetPoints.TwistSetpoints.TWIST_DEFAULT);
        break;
    }
  }
}