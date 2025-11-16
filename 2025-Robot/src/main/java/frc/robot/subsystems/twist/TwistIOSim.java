package frc.robot.subsystems.twist;

import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.subsystems.twist.Twist.TwistState;

public class TwistIOSim implements TwistIO {
    DCMotor gearbox = DCMotor.getKrakenX60Foc(1).withReduction(Constants.Ratios.TWIST_GEAR_RATIO_ROTOR);
    DCMotorSim sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1),
            Constants.Physical.TWIST_MOI,
            1.0 / Constants.Ratios.TWIST_GEAR_RATIO_ROTOR),
            gearbox);
    double inputCurrent = 0.0;

    @Override
    public void init() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'init'");
    }

    @Override
    public void updateInputs(TwistState systemState) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
    }

    @Override
    public void setPercent(double percent) {
        double velocity = sim.getAngularVelocity().in(AngularVelocityUnit.combine(Radian, Second));
        double currentRequired = gearbox.getCurrent(velocity, 24 * percent /* volts * (Kv in rad/s/V) = rad/s */);

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

}
