package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.Drive.DriveState;
import frc.robot.tools.math.Vector;

public class DriveIOSim extends DriveIO {

    @Override
    void zeroIMU() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'zeroIMU'");
    }

    @Override
    void setYaw(double degrees) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setYaw'");
    }

    @Override
    Rotation2d getYaw() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getYaw'");
    }

    @Override
    void setWheelsStraight() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setWheelsStraight'");
    }

    @Override
    protected void setCurrentLimits(int supply, int stator) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setCurrentLimits'");
    }

    @Override
    protected void setPosition(Pose2d pose) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPosition'");
    }

    @Override
    protected Pose2d getPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
    }

    @Override
    protected void drive(Vector velocityVector, double turnVelocity) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'drive'");
    }

    @Override
    protected void driveRobotCentric(Vector velocityVector, double turnRadiansPerSec) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'driveRobotCentric'");
    }

    @Override
    protected void driveCamCentric(Vector velocityVector, double turnRadiansPerSec, double camAngle) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'driveCamCentric'");
    }

    @Override
    protected Vector getVelocityVector() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getVelocityVector'");
    }

    @Override
    void update(DriveState currentState) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'update'");
    }

}
