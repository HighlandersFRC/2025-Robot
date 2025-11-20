package frc.robot.subsystems.twist;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.twist.Twist.TwistState;

public class TwistIOSim implements TwistIO {
    DCMotor gearbox = Constants.MotorSpecs.x44.getX44Gearbox(1).withReduction(Constants.Ratios.TWIST_GEAR_RATIO_ROTOR);
    private final Matrix<N2, N2> A = MatBuilder.fill(
            Nat.N2(),
            Nat.N2(),
            0,
            1,
            0,
            -gearbox.KtNMPerAmp / (gearbox.KvRadPerSecPerVolt * gearbox.rOhms * Constants.Physical.TWIST_MOI));
    private final Vector<N2> B = VecBuilder.fill(0.0, gearbox.KtNMPerAmp / Constants.Physical.TWIST_MOI);
    double inputTorqueCurrent = 0.0;
    private Vector<N2> simState;

    public TwistIOSim() {
        simState = VecBuilder.fill(Units.rotationsToRadians(Constants.SetPoints.TwistSetpoints.TWIST_SIDE), 0.0);
    }

    @Override
    public void init() {
    }

    @Override
    public void updateInputs(TwistState systemState) {
    }

    @Override
    public void setPercent(double percent) {
        double velocity = simState.get(1);
        double currentRequired = gearbox.getCurrent(velocity, 24 * percent /* volts * (Kv in rad/s/V) = rad/s */);
        inputTorqueCurrent = currentRequired;
    }

    @Override
    public void setTorque(double torque, double maxPercent) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setTorque'");
    }

    @Override
    public double getPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
    }

    @Override
    public void setEncoderPosition(double position) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setEncoderPosition'");
    }

    @Override
    public void setPosition(double rotations, int slot) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPosition'");
    }

    private void update(double dt) {
        inputTorqueCurrent = MathUtil.clamp(inputTorqueCurrent, -120.0, 120.0);
        Matrix<N2, N1> updatedState = NumericalIntegration.rkdp(
                (Matrix<N2, N1> x, Matrix<N1, N1> u) -> A.times(x).plus(B.times(u)),
                simState,
                MatBuilder.fill(Nat.N1(), Nat.N1(), inputTorqueCurrent),
                dt);
        // Apply limits
        simState = VecBuilder.fill(updatedState.get(0, 0), updatedState.get(1, 0));
        if (simState.get(0) <= Units.rotationsToRadians(Constants.SetPoints.TwistSetpoints.TWIST_UP)) {
            simState.set(0, 0, Units.rotationsToRadians(Constants.SetPoints.TwistSetpoints.TWIST_UP));
            simState.set(1, 0, 0.0);
        }
        if (simState.get(0) >= Units.rotationsToRadians(Constants.SetPoints.TwistSetpoints.TWIST_DOWN)) {
            simState.set(0, 0, Units.rotationsToRadians(Constants.SetPoints.TwistSetpoints.TWIST_DOWN));
            simState.set(1, 0, 0.0);
        }
    }
}
