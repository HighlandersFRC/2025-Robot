package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.subsystems.shooter.Shooter.ShooterState;

public class ShooterIOComp implements ShooterIO {

    private final TalonFX shooterMotorMaster = new TalonFX(Constants.CANInfo.SHOOTER_FLYWHEEL_MOTOR_MASTER_ID,
            new CANBus(Constants.CANInfo.CANBUS_NAME));
    private final TalonFX shooterMotorFollower = new TalonFX(Constants.CANInfo.SHOOTER_FLYWHEEL_MOTOR_FOLLOWER_ID,
            new CANBus(Constants.CANInfo.CANBUS_NAME));
    private final TalonFX hoodMotor = new TalonFX(Constants.CANInfo.SHOOTER_HOOD_MOTOR_ID,
            new CANBus(Constants.CANInfo.CANBUS_NAME));

    private final TalonFXConfiguration shooterMotorConfiguration = new TalonFXConfiguration();

    private final double hoodJerk = 0.0;
    private final double hoodAcceleration = 6.0 * Constants.Ratios.Shooter.HOOD_GEAR_RATIO;
    private final double hoodCruiseVelocity = 6.0 * Constants.Ratios.Shooter.HOOD_GEAR_RATIO;;

    private final DynamicMotionMagicVoltage hoodMotionProfileRequest = new DynamicMotionMagicVoltage(0,
            hoodCruiseVelocity,
            hoodAcceleration,
            hoodJerk);

    private final VelocityTorqueCurrentFOC flywheelVelocityRequest = new VelocityTorqueCurrentFOC(0); // rps

    private final double hoodProfileScalarFactor = 1;

    @Override
    public void init() {

        hoodMotor.setNeutralMode(NeutralModeValue.Brake);
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.Slot0.kP = 100.0;
        hoodConfig.Slot0.kI = 0.0;
        hoodConfig.Slot0.kD = 5.0;
        hoodConfig.Slot1.kP = 30.0;
        hoodConfig.Slot1.kI = 0.0;
        hoodConfig.Slot1.kD = 5.0;
        hoodConfig.Slot2.kP = 50.0;
        hoodConfig.Slot2.kI = 0.0;
        hoodConfig.Slot2.kD = 15.0;
        hoodConfig.MotionMagic.MotionMagicJerk = this.hoodJerk;
        hoodConfig.MotionMagic.MotionMagicAcceleration = this.hoodAcceleration;
        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = this.hoodCruiseVelocity;
        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.StatorCurrentLimit = 40;
        hoodConfig.CurrentLimits.SupplyCurrentLimit = 40;
        hoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        hoodConfig.Feedback.SensorToMechanismRatio = 1.0;
        hoodConfig.Feedback.RotorToSensorRatio = Constants.Ratios.Shooter.HOOD_GEAR_RATIO;
        hoodMotor.getConfigurator().apply(hoodConfig);
        hoodMotor.setNeutralMode(NeutralModeValue.Brake);

        shooterMotorConfiguration.Slot0.kP = 12;
        shooterMotorConfiguration.Slot0.kI = 0;
        shooterMotorConfiguration.Slot0.kD = 0;
        shooterMotorConfiguration.Slot0.kS = 1;
        shooterMotorConfiguration.Slot0.kV = 0.2;
        shooterMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 140;
        shooterMotorConfiguration.CurrentLimits.StatorCurrentLimit = 140;
        shooterMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterMotorMaster.getConfigurator().apply(shooterMotorConfiguration);
        shooterMotorMaster.setNeutralMode(NeutralModeValue.Coast);
        shooterMotorFollower.getConfigurator().apply(shooterMotorConfiguration);
        shooterMotorFollower.setNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public void updateInputs(ShooterState systemState) {

    }

    @Override
    public void setShooterPercent(double percent) {
        shooterMotorMaster.set(percent);
        shooterMotorFollower.set(-percent);
    }

    @Override
    public void setShooterRPM(double rpm) {
        shooterMotorMaster.setControl(flywheelVelocityRequest
                .withVelocity(Constants.RPMToRPS(rpm) * Constants.Ratios.Shooter.SHOOTER_GEAR_RATIO));
        shooterMotorFollower.setControl(flywheelVelocityRequest
                .withVelocity(Constants.RPMToRPS(-rpm) * Constants.Ratios.Shooter.SHOOTER_GEAR_RATIO));

    }

    @Override
    public void setHoodAngle(double angleDegrees) {

        hoodMotor.setControl(this.hoodMotionProfileRequest
                .withPosition(
                        angleDegrees / 360.0)
                .withVelocity(this.hoodCruiseVelocity * hoodProfileScalarFactor)
                .withAcceleration(this.hoodAcceleration * hoodProfileScalarFactor)
                .withJerk(
                        this.hoodJerk * hoodProfileScalarFactor)
                .withSlot(0));
    }

    @Override
    public double getShooterRPM() {
        return shooterMotorMaster.getRotorVelocity().getValueAsDouble() * 60.0
                / Constants.Ratios.Shooter.SHOOTER_GEAR_RATIO;
    }

    @Override
    public double getHoodAngle() {
        return hoodMotor.getPosition().getValueAsDouble() * 360.0;
    }

    @Override
    public double getShooterStatorCurrent() {
        return shooterMotorMaster.getStatorCurrent().getValueAsDouble();
    }
}
