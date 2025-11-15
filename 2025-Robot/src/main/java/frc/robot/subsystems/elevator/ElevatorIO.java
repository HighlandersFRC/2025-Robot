package frc.robot.subsystems.elevator;

import frc.robot.subsystems.elevator.Elevator.ElevatorState;

public interface ElevatorIO {

    public void updateInputs(ElevatorState systemState);

    public void teleopInit();

    public void autoInit();

    public void setCurrentLimit(double stator, double supply);

    public void init();

    public void moveWithPercent(double percent);

    public void moveWithTorque(double current, double maxPercent);

    public void moveElevatorToPosition(double position);

    public void moveElevatorToPositionSlow(double position);

    public double getElevatorPosition();

    public void setElevatorEncoderPosition(double position);

    public boolean getZeroed();

    public double getVelocity();

}
