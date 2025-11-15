package frc.robot.subsystems.elevator;

import frc.robot.subsystems.elevator.Elevator.ElevatorState;

public class ElevatorIOSim implements ElevatorIO {

    @Override
    public void teleopInit() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'teleopInit'");
    }

    @Override

    public void updateInputs(ElevatorState systemState) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
    }

    @Override
    public void autoInit() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'autoInit'");
    }

    @Override
    public void setCurrentLimit(double stator, double supply) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setCurrentLimit'");
    }

    @Override
    public void init() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'init'");
    }

    @Override
    public void moveWithPercent(double percent) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'moveWithPercent'");
    }

    @Override
    public void moveWithTorque(double current, double maxPercent) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'moveWithTorque'");
    }

    @Override
    public void setElevatorPosition(double position, int slot) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setElevatorPosition'");
    }

    @Override
    public double getElevatorPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getElevatorPosition'");
    }

    @Override
    public void setElevatorEncoderPosition(double position) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setElevatorEncoderPosition'");
    }

    @Override
    public double getCurrent() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getCurrent'");
    }

    @Override
    public double getVelocity() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getVelocity'");
    }

}