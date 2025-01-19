package frc.robot.subsystems;

import org.json.JSONArray;
import org.json.JSONObject;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.tools.controlloops.PID;
import frc.robot.tools.math.Vector;

// **Zero Wheels with the bolt head showing on the left when the front side(battery) is facing down/away from you**

public class Drive extends SubsystemBase {
  private final TalonFX frontRightDriveMotor = new TalonFX(Constants.CANInfo.FRONT_RIGHT_DRIVE_MOTOR_ID,
      Constants.CANInfo.CANBUS_NAME);
  private final TalonFX frontRightAngleMotor = new TalonFX(Constants.CANInfo.FRONT_RIGHT_ANGLE_MOTOR_ID,
      Constants.CANInfo.CANBUS_NAME);
  private final TalonFX frontLeftDriveMotor = new TalonFX(Constants.CANInfo.FRONT_LEFT_DRIVE_MOTOR_ID,
      Constants.CANInfo.CANBUS_NAME);
  private final TalonFX frontLeftAngleMotor = new TalonFX(Constants.CANInfo.FRONT_LEFT_ANGLE_MOTOR_ID,
      Constants.CANInfo.CANBUS_NAME);
  private final TalonFX backLeftDriveMotor = new TalonFX(Constants.CANInfo.BACK_LEFT_DRIVE_MOTOR_ID,
      Constants.CANInfo.CANBUS_NAME);
  private final TalonFX backLeftAngleMotor = new TalonFX(Constants.CANInfo.BACK_LEFT_ANGLE_MOTOR_ID,
      Constants.CANInfo.CANBUS_NAME);
  private final TalonFX backRightDriveMotor = new TalonFX(Constants.CANInfo.BACK_RIGHT_DRIVE_MOTOR_ID,
      Constants.CANInfo.CANBUS_NAME);
  private final TalonFX backRightAngleMotor = new TalonFX(Constants.CANInfo.BACK_RIGHT_ANGLE_MOTOR_ID,
      Constants.CANInfo.CANBUS_NAME);

  private final CANcoder frontRightCanCoder = new CANcoder(Constants.CANInfo.FRONT_RIGHT_MODULE_CANCODER_ID,
      Constants.CANInfo.CANBUS_NAME);
  private final CANcoder frontLeftCanCoder = new CANcoder(Constants.CANInfo.FRONT_LEFT_MODULE_CANCODER_ID,
      Constants.CANInfo.CANBUS_NAME);
  private final CANcoder backLeftCanCoder = new CANcoder(Constants.CANInfo.BACK_LEFT_MODULE_CANCODER_ID,
      Constants.CANInfo.CANBUS_NAME);
  private final CANcoder backRightCanCoder = new CANcoder(Constants.CANInfo.BACK_RIGHT_MODULE_CANCODER_ID,
      Constants.CANInfo.CANBUS_NAME);

  public boolean getSwerveCAN() { // checks to see if all of the swerve motors and encoders are connected
    if (getSwerveMotorsConnected() == 8 && getSwerveCANCodersConnected() == 4) {
      return true;
    } else
      return false;
  }

  public int getSwerveMotorsConnected() { // counts all of the swerve motors that are connected
    int count = 0;
    if (frontRightDriveMotor.clearStickyFault_BootDuringEnable() == StatusCode.OK) {
      count++;
    }
    if (frontLeftDriveMotor.clearStickyFault_BootDuringEnable() == StatusCode.OK) {
      count++;
    }
    if (backRightDriveMotor.clearStickyFault_BootDuringEnable() == StatusCode.OK) {
      count++;
    }
    if (backLeftDriveMotor.clearStickyFault_BootDuringEnable() == StatusCode.OK) {
      count++;
    }
    if (frontRightAngleMotor.clearStickyFault_BootDuringEnable() == StatusCode.OK) {
      count++;
    }
    if (frontLeftAngleMotor.clearStickyFault_BootDuringEnable() == StatusCode.OK) {
      count++;
    }
    if (backRightAngleMotor.clearStickyFault_BootDuringEnable() == StatusCode.OK) {
      count++;
    }
    if (backLeftAngleMotor.clearStickyFault_BootDuringEnable() == StatusCode.OK) {
      count++;
    }
    return count;
  }

  public int getSwerveCANCodersConnected() { // counts all of the swerve encoders that are connected
    int count = 0;
    if (frontRightCanCoder.clearStickyFault_BadMagnet() == StatusCode.OK) {
      count++;
    }
    if (frontLeftCanCoder.clearStickyFault_BadMagnet() == StatusCode.OK) {
      count++;
    }
    if (backRightCanCoder.clearStickyFault_BadMagnet() == StatusCode.OK) {
      count++;
    }
    if (backLeftCanCoder.clearStickyFault_BadMagnet() == StatusCode.OK) {
      count++;
    }
    return count;
  }

  // creates all 4 modules
  private final SwerveModule frontRight = new SwerveModule(1, frontRightAngleMotor, frontRightDriveMotor,
      frontRightCanCoder);
  private final SwerveModule frontLeft = new SwerveModule(2, frontLeftAngleMotor, frontLeftDriveMotor,
      frontLeftCanCoder);
  private final SwerveModule backLeft = new SwerveModule(3, backLeftAngleMotor, backLeftDriveMotor, backLeftCanCoder);
  private final SwerveModule backRight = new SwerveModule(4, backRightAngleMotor, backRightDriveMotor,
      backRightCanCoder);

  Peripherals peripherals;

  // xy position of module based on robot width and distance from edge of robot
  private final double moduleX = ((Constants.Physical.ROBOT_LENGTH) / 2) - Constants.Physical.MODULE_OFFSET;
  private final double moduleY = ((Constants.Physical.ROBOT_WIDTH) / 2) - Constants.Physical.MODULE_OFFSET;

  // Locations for the swerve drive modules relative to the robot center.
  Translation2d m_frontLeftLocation = new Translation2d(moduleX, moduleY);
  Translation2d m_frontRightLocation = new Translation2d(moduleX, -moduleY);
  Translation2d m_backLeftLocation = new Translation2d(-moduleX, moduleY);
  Translation2d m_backRightLocation = new Translation2d(-moduleX, -moduleY);

  // odometry
  private double m_currentX = 0;
  private double m_currentY = 0;
  private double m_currentTheta = 0;

  private double m_averagedX = 0.0;
  private double m_averagedY = 0.0;
  private double m_averagedTheta = 0.0;

  private double m_initTime;
  private double m_currentTime;

  // array for fused odometry
  private double[] currentFusedOdometry = new double[3];

  // Creating my kinematics object using the module locations
  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  SwerveDrivePoseEstimator m_odometry;
  Pose2d m_pose;

  SwerveDrivePoseEstimator loggingOdometry;
  Pose2d loggingPose;

  SwerveDrivePoseEstimator mt2Odometry;
  Pose2d mt2Pose;

  double initAngle;
  double setAngle;
  double diffAngle;

  // path following PID values
  private double kXP = 4.00;
  private double kXI = 0.00;
  private double kXD = 1.20;

  private double kYP = 4.00;
  private double kYI = 0.00;
  private double kYD = 1.20;

  private double kThetaP = 2.70;
  private double kThetaI = 0.00;
  private double kThetaD = 2.00;

  // auto placement PID values
  private double kkXP = 3.00;
  private double kkXI = 0.00;
  private double kkXD = 1.60;

  private double kkYP = 3.00;
  private double kkYI = 0.00;
  private double kkYD = 1.60;

  private double kkThetaP = 2.70;
  private double kkThetaI = 0.00;
  private double kkThetaD = 2.00;

  // teleop targeting PID values
  private double kTurningP = 0.04;
  private double kTurningI = 0;
  private double kTurningD = 0.06;

  private PID xxPID = new PID(kkXP, kkXI, kkXD);
  private PID yyPID = new PID(kkYP, kkYI, kkYD);
  private PID thetaaPID = new PID(kkThetaP, kkThetaI, kkThetaD);

  private PID xPID = new PID(kXP, kXI, kXD);
  private PID yPID = new PID(kYP, kYI, kYD);
  private PID thetaPID = new PID(kThetaP, kThetaI, kThetaD);
  private PID turningPID = new PID(kTurningP, kTurningI, kTurningD);

  private String m_fieldSide = "blue";

  private double angleSetpoint = 0;
  private double teleopInitTime = 0;

  public enum DriveState {
    DEFAULT,
    IDLE,
  }

  private DriveState wantedState = DriveState.IDLE;
  private DriveState systemState = DriveState.IDLE;

  /**
   * Creates a new instance of the Swerve Drive subsystem.
   * Initializes the Swerve Drive subsystem with the provided peripherals.
   * 
   * @param peripherals The peripherals used by the Swerve Drive subsystem.
   */
  public Drive(Peripherals peripherals) {
    this.peripherals = peripherals;

    SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
    swerveModulePositions[0] = new SwerveModulePosition(0, new Rotation2d(frontLeft.getCanCoderPositionRadians()));
    swerveModulePositions[1] = new SwerveModulePosition(0, new Rotation2d(frontRight.getCanCoderPositionRadians()));
    swerveModulePositions[2] = new SwerveModulePosition(0, new Rotation2d(backLeft.getCanCoderPositionRadians()));
    swerveModulePositions[3] = new SwerveModulePosition(0, new Rotation2d(backRight.getCanCoderPositionRadians()));

    Pose2d m_pose = new Pose2d();

    m_odometry = new SwerveDrivePoseEstimator(m_kinematics,
        new Rotation2d((Math.toRadians(peripherals.getPigeonAngle()))), swerveModulePositions, m_pose);
    loggingOdometry = new SwerveDrivePoseEstimator(m_kinematics,
        new Rotation2d(Math.toRadians(peripherals.getPigeonAngle())), swerveModulePositions, m_pose);
    mt2Odometry = new SwerveDrivePoseEstimator(m_kinematics,
        new Rotation2d(Math.toRadians(peripherals.getPigeonAngle())), swerveModulePositions, m_pose);
  }

  public boolean atSetpoint() {
    double currentAngle = peripherals.getPigeonAngle();
    if (getFieldSide().equals("red")) {
      currentAngle -= 180;
    }
    return (Math.abs(Constants.standardizeAngleDegrees(currentAngle)
        - Constants.standardizeAngleDegrees(angleSetpoint)) < 2);
  }

  public void setWantedState(DriveState wantedState) {
    this.wantedState = wantedState;
  }

  public void setWantedState(DriveState wantedState, double angle) {
    this.wantedState = wantedState;
    setSetpointAngle(angle);
  }

  public void setSetpointAngle(double angle) {
    this.angleSetpoint = angle;
    Logger.recordOutput("Setpoint Theta: ", angle);
  }

  /**
   * Initializes the robot with the specified field side configuration.
   * It sets up configurations when run on robot initialization, such as setting
   * the field side,
   * initializing each swerve module, configuring motor inversions, and setting
   * PID controller output limits.
   * Additionally, it sets the default command to the DriveDefault command.
   *
   * @param fieldSide The side of the field (e.g., "red" or "blue").
   */
  public void init(String fieldSide) {
    // sets configurations when run on robot initalization
    this.m_fieldSide = fieldSide;

    frontRight.init();
    frontLeft.init();
    backRight.init();
    backLeft.init();

    xxPID.setMinOutput(-4.9);
    xxPID.setMaxOutput(4.9);

    yyPID.setMinOutput(-4.9);
    yyPID.setMaxOutput(4.9);

    thetaaPID.setMinOutput(-3);
    thetaaPID.setMaxOutput(3);

    xPID.setMinOutput(-4.9);
    xPID.setMaxOutput(4.9);

    yPID.setMinOutput(-4.9);
    yPID.setMaxOutput(4.9);

    thetaPID.setMinOutput(-3);
    thetaPID.setMaxOutput(3);

    turningPID.setMinOutput(-3);
    turningPID.setMaxOutput(3);
  }

  public void teleopInit() {
    teleopInitTime = Timer.getFPGATimestamp();
    angleSetpoint = peripherals.getPigeonAngle();
    if (getFieldSide() == "red") {
      angleSetpoint -= 180;
    }
    turningPID.setSetPoint(angleSetpoint);
    if (Math.abs(turningPID.getSetPoint() - angleSetpoint) > 100) {
      turningPID.setSetPoint(angleSetpoint);
    }
  }

  public void teleopPeriodic() {
    if (Math.abs(turningPID.getSetPoint() - angleSetpoint) > 100) {
      turningPID.setSetPoint(angleSetpoint);
    }
  }

  /**
   * Zeros the IMU (Inertial Measurement Unit) mid-match and resets the odometry
   * with a zeroed angle.
   * It resets the angle reported by the pigeon sensor to zero and updates the
   * odometry with this new zeroed angle.
   */
  public void zeroIMU() {
    angleSetpoint = 0;
    turningPID.setSetPoint(angleSetpoint);
    peripherals.zeroPigeon();
    // SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
    // swerveModulePositions[0] = new SwerveModulePosition(frontLeft.getModuleDistance(),
    //     new Rotation2d(frontLeft.getCanCoderPositionRadians()));
    // swerveModulePositions[1] = new SwerveModulePosition(frontRight.getModuleDistance(),
    //     new Rotation2d(frontRight.getCanCoderPositionRadians()));
    // swerveModulePositions[2] = new SwerveModulePosition(backLeft.getModuleDistance(),
    //     new Rotation2d(backLeft.getCanCoderPositionRadians()));
    // swerveModulePositions[3] = new SwerveModulePosition(backRight.getModuleDistance(),
    //     new Rotation2d(backRight.getCanCoderPositionRadians()));

    // m_pose = m_odometry.update(new Rotation2d((Math.toRadians(peripherals.getPigeonAngle()))), swerveModulePositions);
    // loggingPose = loggingOdometry.update(new Rotation2d(
    //     Math.toRadians(peripherals.getPigeonAngle())), swerveModulePositions);
    // mt2Pose = mt2Odometry.update(new Rotation2d(Math.toRadians(peripherals.getPigeonAngle())), swerveModulePositions);
  }

  /**
   * Adjusts the angle reported by the pigeon sensor after an autonomous routine.
   * It adds 180 degrees to the current angle reported by the pigeon sensor and
   * wraps it around 360 degrees.
   */
  public void setPigeonAfterAuto() {
    peripherals.setPigeonAngle((peripherals.getPigeonAngle() + 180) % 360);
  }

  /**
   * Sets the angle reported by the pigeon sensor to the specified value.
   *
   * @param angle The angle to set for the pigeon sensor in degrees.
   */
  public void setPigeonAngle(double angle) {
    peripherals.setPigeonAngle(angle);
  }

  /**
   * Retrieves the current angle reported by the pigeon sensor.
   *
   * @return The current angle reported by the pigeon sensor in degrees.
   */
  public double getPigeonAngle() {
    return peripherals.getPigeonAngle();
  }

  /**
   * Sets the PID values for all swerve modules to zero, keeping the wheels
   * straight.
   */
  public void setWheelsStraight() {
    frontRight.setWheelPID(0.0, 0.0);
    frontLeft.setWheelPID(0.0, 0.0);
    backLeft.setWheelPID(0.0, 0.0);
    backRight.setWheelPID(0.0, 0.0);
  }

  /**
   * Initializes the robot's state for autonomous mode based on the provided path
   * points.
   * 
   * @param pathPoints The array of path points representing the trajectory for
   *                   the autonomous routine.
   */
  public void autoInit(JSONArray pathPoints) {
    // runs at start of autonomous
    // System.out.println("Auto init");
    JSONObject firstPoint = pathPoints.getJSONObject(0);
    double firstPointX = firstPoint.getDouble("x");
    double firstPointY = firstPoint.getDouble("y");
    double firstPointAngle = firstPoint.getDouble("angle");

    // changing odometry if on red side, don't need to change y because it will be
    // the same for autos on either side
    if (this.m_fieldSide == "blue") {
      firstPointX = Constants.Physical.FIELD_LENGTH - firstPointX;
      firstPointAngle = Math.PI - firstPointAngle;
    }

    peripherals.setPigeonAngle(Math.toDegrees(firstPointAngle));
    SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
    swerveModulePositions[0] = new SwerveModulePosition(frontLeft.getModuleDistance(),
        new Rotation2d(frontLeft.getCanCoderPositionRadians()));
    swerveModulePositions[1] = new SwerveModulePosition(frontRight.getModuleDistance(),
        new Rotation2d(frontRight.getCanCoderPositionRadians()));
    swerveModulePositions[2] = new SwerveModulePosition(backLeft.getModuleDistance(),
        new Rotation2d(backLeft.getCanCoderPositionRadians()));
    swerveModulePositions[3] = new SwerveModulePosition(backRight.getModuleDistance(),
        new Rotation2d(backRight.getCanCoderPositionRadians()));
    m_odometry.resetPosition(new Rotation2d(firstPointAngle), swerveModulePositions,
        new Pose2d(new Translation2d(firstPointX, firstPointY), new Rotation2d(firstPointAngle)));
    loggingOdometry.resetPosition(new Rotation2d(firstPointAngle), swerveModulePositions,
        new Pose2d(new Translation2d(firstPointX, firstPointY), new Rotation2d(firstPointAngle)));
    mt2Odometry.resetPosition(new Rotation2d(firstPointAngle), swerveModulePositions,
        new Pose2d(new Translation2d(firstPointX, firstPointY), new Rotation2d(firstPointAngle)));

    currentFusedOdometry[0] = firstPointX;
    currentFusedOdometry[1] = firstPointY;
    currentFusedOdometry[2] = firstPointAngle;

    m_currentX = currentFusedOdometry[0];
    m_currentY = currentFusedOdometry[1];
    m_currentTheta = currentFusedOdometry[2];

    m_initTime = Timer.getFPGATimestamp();

    updateOdometryFusedArray();
  }

  /**
   * Sets the current field side designation.
   * 
   * @param side The field side designation to set, indicating whether the robot
   *             is positioned on the "blue" or "red" side of the field.
   */
  public void setFieldSide(String side) {
    m_fieldSide = side;
  }

  /**
   * Retrieves the current field side designation.
   * 
   * @return The current field side designation, indicating whether the robot is
   *         positioned on the "blue" or "red" side of the field.
   */
  public String getFieldSide() {
    return m_fieldSide;
  }

  /**
   * Retrieves the current timestamp relative to the start of the robot operation.
   * 
   * @return The current timestamp in seconds since the start of the robot
   *         operation.
   */
  public double getCurrentTime() {
    return m_currentTime;
  }

  public void addVisionMeasurementToOdometry(Pose2d visionPose, double timestamp) {
    m_odometry.addVisionMeasurement(
        visionPose,
        timestamp);
  }

  public void setOdometry(Pose2d pose) {
    Logger.recordOutput("Odometry Reset to:", pose.toString());
    System.out.println("New Odometry Pose: " + pose.toString());
    m_odometry.resetPose(pose);
    loggingOdometry.resetPose(pose);
    mt2Odometry.resetPose(pose);
  }

  public boolean isPoseInField(Pose2d pose) {
    if (pose.getY() < 0 || pose.getY() > Constants.Physical.FIELD_WIDTH || pose.getX() < 0
        || pose.getX() > Constants.Physical.FIELD_LENGTH) {
      return false;
    } else {
      return true;
    }
  }

  /**
   * Updates the fused odometry array with current robot position and orientation
   * information.
   * Calculates the robot's position and orientation using swerve module positions
   * and the gyro angle.
   * Updates the current X, Y, and theta values, as well as previous values and
   * time differences.
   */
  public void updateOdometryFusedArray() {
    double navxOffset = Math.toRadians(peripherals.getPigeonAngle());

    SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
    swerveModulePositions[0] = new SwerveModulePosition(frontLeft.getModuleDistance(),
        new Rotation2d(frontLeft.getCanCoderPositionRadians()));
    swerveModulePositions[1] = new SwerveModulePosition(frontRight.getModuleDistance(),
        new Rotation2d(frontRight.getCanCoderPositionRadians()));
    swerveModulePositions[2] = new SwerveModulePosition(backLeft.getModuleDistance(),
        new Rotation2d(backLeft.getCanCoderPositionRadians()));
    swerveModulePositions[3] = new SwerveModulePosition(backRight.getModuleDistance(),
        new Rotation2d(backRight.getCanCoderPositionRadians()));

    m_pose = m_odometry.update(new Rotation2d((navxOffset)), swerveModulePositions);
    loggingPose = loggingOdometry.update(new Rotation2d(navxOffset), swerveModulePositions);
    mt2Pose = mt2Odometry.update(new Rotation2d(navxOffset), swerveModulePositions);

    m_currentX = getOdometryX();
    m_currentY = getOdometryY();
    m_currentTheta = navxOffset;
    Pose2d defaultPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));

    // Pose2d frontCamTrigPose = peripherals.getFrontCamTrigPose(); //TODO:
    // uncomment when using camera
    // if (isPoseInField(frontCamTrigPose) && !frontCamTrigPose.equals(defaultPose))
    // {
    // mt2Odometry.addVisionMeasurement(frontCamTrigPose,
    // peripherals.getFrontCamLatency());
    // }

    Pose2d frontCamPnPPose = peripherals.getFrontCamPnPPose().toPose2d();
    if (isPoseInField(frontCamPnPPose) && !frontCamPnPPose.equals(defaultPose)) {
      // peripherals.setPigeonAngle(frontCamPnPPose.getRotation().getRadians());
      mt2Odometry.addVisionMeasurement(frontCamPnPPose,
          peripherals.getFrontCamLatency());
    }

    m_currentTime = Timer.getFPGATimestamp() - m_initTime;

    m_averagedX = (m_currentX + m_averagedX) / 2;
    m_averagedY = (m_currentY + m_averagedY) / 2;
    m_averagedTheta = (m_currentTheta + m_averagedTheta) / 2;

    currentFusedOdometry[0] = m_averagedX;
    currentFusedOdometry[1] = m_averagedY;
    currentFusedOdometry[2] = m_currentTheta;
  }

  /**
   * Retrieves the states of the modules (position and ground speed) of the
   * robot's swerve drive system.
   *
   * @return An array containing the states of each wheel module, consisting of:
   *         front right module position in degrees,
   *         front right module ground speed in meters per second,
   *         front left module position in degrees,
   *         front left module ground speed in meters per second,
   *         back left module position in degrees,
   *         back left module ground speed in meters per second,
   *         back right module position in degrees,
   *         back right module ground speed in meters per second.
   */
  public double[] getModuleStates() {
    double[] states = {
        frontLeft.getCanCoderPosition() * 360.0, frontLeft.getGroundSpeed(),
        frontRight.getCanCoderPosition() * 360.0, frontRight.getGroundSpeed(),
        backLeft.getCanCoderPosition() * 360.0, backLeft.getGroundSpeed(),
        backRight.getCanCoderPosition() * 360.0, backRight.getGroundSpeed(),
    };
    return states;
  }

  /**
   * Retrieves the setpoints of the modules (angle and drive motors) of the
   * robot's swerve drive system.
   *
   * @return An array containing the setpoints of the angle and drive motors for
   *         each wheel module, in the order:
   *         front right angle motor, front right drive motor,
   *         front left angle motor, front left drive motor,
   *         back left angle motor, back left drive motor,
   *         back right angle motor, back right drive motor.
   */
  public double[] getModuleSetpoints() {
    double[] setpoints = {
        frontLeft.getAngleMotorSetpoint() * 360, frontLeft.getDriveMotorSetpoint(),
        frontRight.getAngleMotorSetpoint() * 360, frontRight.getDriveMotorSetpoint(),
        backLeft.getAngleMotorSetpoint() * 360, backLeft.getDriveMotorSetpoint(),
        backRight.getAngleMotorSetpoint() * 360, backRight.getDriveMotorSetpoint(),
    };
    return setpoints;
  }

  /**
   * Retrieves the current velocity of the angle motors of the robot.
   *
   * @return An array containing the current velocity of the angle motors for
   *         front right, front left, back left, and back right wheels.
   */
  public double[] getAngleMotorVelocity() {
    double[] velocity = {
        frontRight.getAngleVelocity(), frontLeft.getAngleVelocity(), backLeft.getAngleVelocity(),
        backRight.getAngleVelocity()
    };
    return velocity;
  }

  /**
   * Retrieves the current odometry information of the robot.
   *
   * @return An array containing the current X-coordinate, Y-coordinate, and
   *         orientation angle of the robot.
   */
  public double[] getOdometry() {
    double[] odometry = {
        getOdometryX(), getOdometryY(), getOdometryAngle()
    };
    return odometry;
  }

  public double[] getLocalizationOdometry() {
    double[] odometry = {
        getLocalizationOdometryX(), getLocalizationOdometryY(), getLocalizationOdometryAngle()
    };
    return odometry;
  }

  public double[] getMT2Odometry() {
    double[] odometry = {
        getMT2OdometryX(), getMT2OdometryY(), getMT2OdometryAngle()
    };
    return odometry;
  }

  public double[] getReefClosestSetpoint(double[] currentOdometry /* {x, y, thetaRadians} */) {
    double x = currentOdometry[0];
    double y = currentOdometry[1];
    double theta = currentOdometry[2];
    double dist = 100.0;
    double currentDist = 100.0;
    double[] chosenSetpoint = { x, y, theta };
    if (getFieldSide() == "red") {
      for (int i = 0; i < Constants.Physical.redCoralScoringPositions.size(); i++) {
        currentDist = Math.sqrt(Math.pow((x - Constants.Physical.redCoralScoringPositions.get(i).getX()), 2)
            + Math.pow((y - Constants.Physical.redCoralScoringPositions.get(i).getY()), 2));
        if (currentDist < dist) {
          dist = currentDist;
          chosenSetpoint[0] = Constants.Physical.redCoralScoringPositions.get(i).getX();
          chosenSetpoint[1] = Constants.Physical.redCoralScoringPositions.get(i).getY();
          chosenSetpoint[2] = Constants.Physical.redCoralScoringPositions.get(i).getRotation().getRadians();
        }
      }
    } else {
      for (int i = 0; i < Constants.Physical.blueCoralScoringPositions.size(); i++) {
        currentDist = Math.sqrt(Math.pow((x - Constants.Physical.blueCoralScoringPositions.get(i).getX()), 2)
            + Math.pow((y - Constants.Physical.blueCoralScoringPositions.get(i).getY()), 2));
        if (currentDist < dist) {
          dist = currentDist;
          chosenSetpoint[0] = Constants.Physical.redCoralScoringPositions.get(i).getX();
          chosenSetpoint[1] = Constants.Physical.redCoralScoringPositions.get(i).getY();
          chosenSetpoint[2] = Constants.Physical.redCoralScoringPositions.get(i).getRotation().getRadians();
        }
      }
    }
    if(Math.hypot(x - getMT2OdometryX(), y - getMT2OdometryY()) > 5) {
      return getMT2Odometry();
    } else {
      return chosenSetpoint;
    }
  }

  /**
   * Retrieves the current X-coordinate of the robot from fused odometry.
   *
   * @return The current X-coordinate of the robot.
   */
  public double getFusedOdometryX() {
    return currentFusedOdometry[0];
  }

  /**
   * Retrieves the current Y-coordinate of the robot from fused odometry.
   *
   * @return The current Y-coordinate of the robot.
   */
  public double getFusedOdometryY() {
    return currentFusedOdometry[1];
  }

  /**
   * Retrieves the current orientation angle of the robot from fused odometry.
   *
   * @return The current orientation angle of the robot in radians.
   */
  public double getFusedOdometryTheta() {
    return currentFusedOdometry[2];
  }

  /**
   * Retrieves the current X-coordinate of the robot from odometry.
   *
   * @return The current X-coordinate of the robot.
   */
  public double getOdometryX() {
    return m_odometry.getEstimatedPosition().getX();
  }

  public double getLocalizationOdometryX() {
    return loggingOdometry.getEstimatedPosition().getX();
  }

  public double getMT2OdometryX() {
    return mt2Odometry.getEstimatedPosition().getX();
  }

  /**
   * Retrieves the current Y-coordinate of the robot from odometry.
   *
   * @return The current Y-coordinate of the robot.
   */
  public double getOdometryY() {
    return m_odometry.getEstimatedPosition().getY();
  }

  public double getLocalizationOdometryY() {
    return loggingOdometry.getEstimatedPosition().getY();
  }

  public double getMT2OdometryY() {
    return mt2Odometry.getEstimatedPosition().getY();
  }

  /**
   * Retrieves the current orientation angle of the robot from odometry.
   *
   * @return The current orientation angle of the robot in radians.
   */
  public double getOdometryAngle() {
    return m_odometry.getEstimatedPosition().getRotation().getRadians();
  }

  public double getLocalizationOdometryAngle() {
    return loggingOdometry.getEstimatedPosition().getRotation().getRadians();
  }

  public double getMT2OdometryAngle() {
    return mt2Odometry.getEstimatedPosition().getRotation().getRadians();
  }

  /**
   * Drives the robot with alignment adjustment based on the specified angle from
   * placement.
   * 
   * @param degreesFromPlacement The angle in degrees from the placement
   *                             orientation to align with.
   */
  public void driveAutoAligned(double degreesFromPlacement) {
    updateOdometryFusedArray();

    double turn = degreesFromPlacement;

    double originalX = -(Math.copySign(OI.getDriverLeftY() * OI.getDriverLeftY(), OI.getDriverLeftY()));
    double originalY = -(Math.copySign(OI.getDriverLeftX() * OI.getDriverLeftX(), OI.getDriverLeftX()));

    if (Math.abs(originalX) < 0.05) {
      originalX = 0;
    }
    if (Math.abs(originalY) < 0.05) {
      originalY = 0;
    }

    double pigeonAngle = Math.toRadians(peripherals.getPigeonAngle());
    double xPower = getAdjustedX(originalX, originalY);
    double yPower = getAdjustedY(originalX, originalY);

    double xSpeed = xPower * Constants.Physical.TOP_SPEED;
    double ySpeed = yPower * Constants.Physical.TOP_SPEED;

    Vector controllerVector = new Vector(xSpeed, ySpeed);

    frontLeft.drive(controllerVector, turn, pigeonAngle);
    frontRight.drive(controllerVector, turn, pigeonAngle);
    backLeft.drive(controllerVector, turn, pigeonAngle);
    backRight.drive(controllerVector, turn, pigeonAngle);
  }

  /**
   * Turns the robot in robot-centric mode.
   * 
   * @param turn The rate at which the robot should turn in radians per second.
   */
  public void autoRobotCentricTurn(double turn) {
    frontLeft.drive(new Vector(0, 0), turn, 0.0);
    frontRight.drive(new Vector(0, 0), turn, 0.0);
    backLeft.drive(new Vector(0, 0), turn, 0.0);
    backRight.drive(new Vector(0, 0), turn, 0.0);
    updateOdometryFusedArray();
  }

  /**
   * Drives the robot in robot-centric mode using velocity vector and turning
   * rate.
   * 
   * @param velocityVector    The velocity vector containing x and y velocities in
   *                          meters per second (m/s).
   * @param turnRadiansPerSec The rate at which the robot should spin in radians
   *                          per second.
   */
  public void autoRobotCentricDrive(Vector velocityVector, double turnRadiansPerSec) {
    updateOdometryFusedArray();
    frontLeft.drive(velocityVector, turnRadiansPerSec, 0);
    frontRight.drive(velocityVector, turnRadiansPerSec, 0);
    backLeft.drive(velocityVector, turnRadiansPerSec, 0);
    backRight.drive(velocityVector, turnRadiansPerSec, 0);
  }

  /**
   * Drives the robot during teleoperation.
   * 
   * @apiNote This method updates the fused odometry array and controls the
   *          robot's movement based on joystick inputs.
   */
  public void teleopDrive() {
    updateOdometryFusedArray();
    double turnLimit = 0.17;
    // 0.35 before

    if (OI.driverController.getLeftBumper()) {
      // activate speedy spin
      turnLimit = 1;
    }

    // this is correct, X is forward in field, so originalX should be the y on the
    // joystick
    double originalX = -(Math.copySign(OI.getDriverLeftY() * OI.getDriverLeftY(), OI.getDriverLeftY()));
    double originalY = -(Math.copySign(OI.getDriverLeftX() * OI.getDriverLeftX(), OI.getDriverLeftX()));
    if (Math.abs(originalX) < 0.075) {
      originalX = 0;
    }
    if (Math.abs(originalY) < 0.075) {
      originalY = 0;
    }

    // double turn = turnLimit * ((Math.copySign(OI.getDriverRightX() *
    // OI.getDriverRightX() * OI.getDriverRightX(), OI.getDriverRightX())) *
    // (Constants.Physical.TOP_SPEED)/(Constants.Physical.ROBOT_RADIUS));
    double turn = turnLimit
        * (OI.getDriverRightX() * (Constants.Physical.TOP_SPEED) / (Constants.Physical.ROBOT_RADIUS));

    if (Math.abs(turn) < 0.15) {
      turn = 0.0;
    }

    if (turn == 0.0 && Timer.getFPGATimestamp() - teleopInitTime > 2.0) {
      turningPID.setSetPoint(angleSetpoint);
      double yaw = peripherals.getPigeonAngle();
      while (Math.abs(angleSetpoint - yaw) > 180) {
        if (angleSetpoint - yaw > 180) {
          yaw += 360;
        } else {
          yaw -= 360;
        }
      }
      double result = -2 * turningPID.updatePID(yaw);
      Logger.recordOutput("result", result);
      driveAutoAligned(result);
    } else {
      angleSetpoint = peripherals.getPigeonAngle();
      double compensation = peripherals.getPigeonAngularVelocityW() * 0.050;
      angleSetpoint += compensation;
      Logger.recordOutput("setpoint", angleSetpoint);
      turningPID.setSetPoint(angleSetpoint);
      double pigeonAngle = Math.toRadians(peripherals.getPigeonAngle());
      double xPower = getAdjustedX(originalX, originalY);
      double yPower = getAdjustedY(originalX, originalY);

      double xSpeed = xPower * Constants.Physical.TOP_SPEED;
      double ySpeed = yPower * Constants.Physical.TOP_SPEED;

      Vector controllerVector = new Vector(xSpeed, ySpeed);
      frontLeft.drive(controllerVector, turn, pigeonAngle);
      frontRight.drive(controllerVector, turn, pigeonAngle);
      backLeft.drive(controllerVector, turn, pigeonAngle);
      backRight.drive(controllerVector, turn, pigeonAngle);
    }
  }

  public void teleopDriveToPiece(double yToPiece) {
    updateOdometryFusedArray();
    double turnLimit = 0.17;
    double kP = 0.8;
    // 0.35 before

    if (OI.driverController.getLeftBumper()) {
      // activate speedy spin
      turnLimit = 1;
    }

    // this is correct, X is forward in field, so originalX should be the y on the
    // joystick
    double originalX = -(Math.copySign(OI.getDriverLeftY() * OI.getDriverLeftY(), OI.getDriverLeftY()));
    double originalY = yToPiece * kP;

    if (Math.abs(originalX) < 0.075) {
      originalX = 0;
    }

    double turn = turnLimit
        * (OI.getDriverRightX() * (Constants.Physical.TOP_SPEED) / (Constants.Physical.ROBOT_RADIUS));

    if (Math.abs(turn) < 0.15) {
      turn = 0.0;
    }

    if (turn == 0.0) {
      turningPID.setSetPoint(angleSetpoint);
      double yaw = peripherals.getPigeonAngle();
      while (Math.abs(angleSetpoint - yaw) > 180) {
        if (angleSetpoint - yaw > 180) {
          yaw += 360;
        } else {
          yaw -= 360;
        }
      }
      double result = -2 * turningPID.updatePID(yaw);
      Logger.recordOutput("result", result);
      updateOdometryFusedArray();

      double x = -(Math.copySign(OI.getDriverLeftY() * OI.getDriverLeftY(), OI.getDriverLeftY()));
      double y = yToPiece * kP;

      if (Math.abs(originalX) < 0.05) {
        originalX = 0;
      }

      double pigeonAngle = Math.toRadians(peripherals.getPigeonAngle());
      double xPower = getAdjustedX(x, y);
      double yPower = getAdjustedY(x, y);

      double xSpeed = xPower * Constants.Physical.TOP_SPEED;
      double ySpeed = yPower * Constants.Physical.TOP_SPEED;

      Vector controllerVector = new Vector(xSpeed, ySpeed);

      frontLeft.drive(controllerVector, result, pigeonAngle);
      frontRight.drive(controllerVector, result, pigeonAngle);
      backLeft.drive(controllerVector, result, pigeonAngle);
      backRight.drive(controllerVector, result, pigeonAngle);
    } else {
      angleSetpoint = peripherals.getPigeonAngle();
      double compensation = peripherals.getPigeonAngularVelocityW() * 0.050;
      angleSetpoint += compensation;
      Logger.recordOutput("setpoint", angleSetpoint);
      turningPID.setSetPoint(angleSetpoint);
      double pigeonAngle = Math.toRadians(peripherals.getPigeonAngle());
      double xPower = getAdjustedX(originalX, originalY);
      double yPower = getAdjustedY(originalX, originalY);

      double xSpeed = xPower * Constants.Physical.TOP_SPEED;
      double ySpeed = yPower * Constants.Physical.TOP_SPEED;

      Vector controllerVector = new Vector(xSpeed, ySpeed);
      frontLeft.drive(controllerVector, turn, pigeonAngle);
      frontRight.drive(controllerVector, turn, pigeonAngle);
      backLeft.drive(controllerVector, turn, pigeonAngle);
      backRight.drive(controllerVector, turn, pigeonAngle);
    }
  }

  public void driveToPoint(double x, double y, double theta) {
    
    Logger.recordOutput("Magnitude Error Inches", Constants.metersToInches(Math.sqrt(Math.pow(x - getMT2OdometryX(), 2) + Math.pow(y - getMT2OdometryY(), 2))));
    Logger.recordOutput("Theta Error Degrees", Math.toDegrees(theta - getMT2OdometryAngle()));

    while (Math.abs(theta - getMT2OdometryAngle()) > Math.PI) {
      if (theta - getMT2OdometryAngle() > Math.PI) {
        theta -= 2 * Math.PI;
      } else {
        theta += 2 * Math.PI;
      }
    }

    xxPID.setSetPoint(x);
    yyPID.setSetPoint(y);
    thetaaPID.setSetPoint(theta);

    xxPID.updatePID(getMT2OdometryX());
    yyPID.updatePID(getMT2OdometryY());
    thetaaPID.updatePID(getMT2OdometryAngle());

    double xVelNoFF = xxPID.getResult();
    double yVelNoFF = yyPID.getResult();
    double thetaVelNoFF = -thetaaPID.getResult();

    // double feedForwardX = targetPoint.getDouble("x_velocity") *
    // Constants.Autonomous.FEED_FORWARD_MULTIPLIER;
    // double feedForwardY = targetPoint.getDouble("y_velocity") *
    // Constants.Autonomous.FEED_FORWARD_MULTIPLIER;
    // double feedForwardTheta = -targetPoint.getDouble("angular_velocity") *
    // Constants.Autonomous.FEED_FORWARD_MULTIPLIER;

    double finalX = xVelNoFF;
    double finalY = yVelNoFF;
    double finalTheta = thetaVelNoFF;
    if (m_fieldSide == "blue") {
      finalX = -finalX;
      finalTheta = -finalTheta;
    }
    Number[] velocityArray = new Number[] {
        finalX,
        -finalY,
        finalTheta,
    };

    Vector velocityVector = new Vector();
    double desiredThetaChange = 0;
    velocityVector.setI(velocityArray[0].doubleValue());
    velocityVector.setJ(velocityArray[1].doubleValue());
    desiredThetaChange = velocityArray[2].doubleValue();

    autoDrive(velocityVector, desiredThetaChange);


  }

  /**
   * Runs autonomous driving by providing velocity vector and turning rate.
   * 
   * @param vector            The velocity vector containing xy velocities.
   * @param turnRadiansPerSec The rate at which the robot should spin in radians
   *                          per second.
   */
  public void autoDrive(Vector vector, double turnRadiansPerSec) {
    updateOdometryFusedArray();

    double pigeonAngle = Math.toRadians(peripherals.getPigeonAngle());

    double[] odometryList = new double[3];

    odometryList[0] = getFusedOdometryX();
    odometryList[1] = getFusedOdometryY();
    odometryList[2] = getFusedOdometryTheta();

    frontLeft.drive(vector, turnRadiansPerSec, pigeonAngle);
    frontRight.drive(vector, turnRadiansPerSec, pigeonAngle);
    backLeft.drive(vector, turnRadiansPerSec, pigeonAngle);
    backRight.drive(vector, turnRadiansPerSec, pigeonAngle);
  }

  /**
   * Retrieves the current velocity vector of the robot in field coordinates.
   * The velocity vector is calculated based on the individual wheel speeds and
   * orientations.
   *
   * @return The current velocity vector of the robot in meters per second (m/s).
   */
  public Vector getRobotVelocityVector() {
    Vector velocityVector = new Vector(0, 0);
    double pigeonAngleRadians = Math.toRadians(this.peripherals.getPigeonAngle());

    double frV = this.frontRight.getGroundSpeed();
    double frTheta = this.frontRight.getWheelPosition() + pigeonAngleRadians;
    double frVX = frV * Math.cos(frTheta);
    double frVY = frV * Math.sin(frTheta);
    double flV = this.frontLeft.getGroundSpeed();
    double flTheta = this.frontLeft.getWheelPosition() + pigeonAngleRadians;
    double flVX = flV * Math.cos(flTheta);
    double flVY = flV * Math.sin(flTheta);
    double blV = this.backLeft.getGroundSpeed();
    double blTheta = this.backLeft.getWheelPosition() + pigeonAngleRadians;
    double blVX = blV * Math.cos(blTheta);
    double blVY = blV * Math.sin(blTheta);
    double brV = this.backRight.getGroundSpeed();
    double brTheta = this.backRight.getWheelPosition() + pigeonAngleRadians;
    double brVX = brV * Math.cos(brTheta);
    double brVY = brV * Math.sin(brTheta);

    velocityVector.setI(frVX + flVX + blVX + brVX);
    velocityVector.setJ(frVY + flVY + blVY + brVY);

    // System.out.println("VVector: <" + velocityVector.getI() + ", " +
    // velocityVector.getJ() + ">");

    return velocityVector;
  }

  /**
   * Retrieves the acceleration vector of the robot.
   * The acceleration is calculated based on the linear acceleration measured by
   * the Pigeon IMU.
   * The acceleration is expressed in field-centric coordinates.
   *
   * @return The acceleration vector of the robot in meters per second squared
   *         (m/s^2).
   */
  public Vector getRobotAccelerationVector() {
    Vector accelerationVector = new Vector();

    Vector robotCentricAccelerationVector = this.peripherals.getPigeonLinAccel();
    double accelerationMagnitude = Constants.getDistance(robotCentricAccelerationVector.getI(),
        robotCentricAccelerationVector.getJ(), 0, 0);
    accelerationVector.setI(accelerationMagnitude * Math.cos(Math.toRadians(this.peripherals.getPigeonAngle())));
    accelerationVector.setJ(accelerationMagnitude * Math.sin(Math.toRadians(this.peripherals.getPigeonAngle())));

    return accelerationVector;
  }

  /**
   * Retrieves the path point closest to the specified time from the given path.
   * If the specified time is before the first path point, the first point is
   * returned.
   * If the specified time is after the last path point, the last point is
   * returned.
   *
   * @param path The array containing path points, each represented as a
   *             JSONArray.
   * @param time The time for which the closest path point is required.
   * @return The closest path point to the specified time.
   */
  public JSONArray getPathPoint(JSONArray path, double time) {
    for (int i = 0; i < path.length() - 1; i++) {
      JSONArray currentPoint = path.getJSONArray(i + 1);
      JSONArray previousPoint = path.getJSONArray(i);
      double currentPointTime = currentPoint.getDouble(0);
      double previousPointTime = previousPoint.getDouble(0);
      if (time >= previousPointTime && time < currentPointTime) {
        return currentPoint;
      }
    }
    if (time < path.getJSONArray(0).getDouble(0)) {
      return path.getJSONArray(0);
    } else {
      return path.getJSONArray(path.length() - 1);
    }
  }

  public Number[] purePursuitController(double currentX, double currentY, double currentTheta, int currentIndex,
      JSONArray pathPoints, boolean fullSend, boolean accurate) {
    JSONObject targetPoint = pathPoints.getJSONObject(pathPoints.length() - 1);
    int targetIndex = pathPoints.length() - 1;
    if (this.m_fieldSide == "blue") {
      currentX = Constants.Physical.FIELD_LENGTH - currentX;
      currentTheta = Math.PI - currentTheta;
    }
    for (int i = currentIndex; i < pathPoints.length(); i++) {
      JSONObject point = pathPoints.getJSONObject(i);
      double targetX = point.getDouble("x"), targetY = point.getDouble("y"),
          targetTheta = point.getDouble("angle"), targetXvel = point.getDouble("x_velocity"),
          targetYvel = point.getDouble("y_velocity"), targetThetavel = point.getDouble("angular_velocity");
      while (Math.abs(targetTheta - currentTheta) > Math.PI) {
        if (targetTheta - currentTheta > Math.PI) {
          targetTheta -= 2 * Math.PI;
        } else if (targetTheta - currentTheta < -Math.PI) {
          targetTheta += 2 * Math.PI;
        }
      }
      double linearVelMag = Math.hypot(targetYvel / Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS,
          targetXvel / Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS);
      double targetVelMag = Math.hypot(linearVelMag,
          targetThetavel / Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_ANGULAR_RADIUS);
      double lookaheadRadius = fullSend ? Constants.Autonomous.FULL_SEND_LOOKAHEAD
          : Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_DISTANCE * targetVelMag
              + Constants.Autonomous.MIN_LOOKAHEAD_DISTANCE;// If full send mode is enabled, use the full send lookahead
      double deltaX = (currentX - targetX), deltaY = (currentY - targetY), deltaTheta = (currentTheta - targetTheta);
      if (!insideRadius(deltaX / Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS,
          deltaY / Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS,
          deltaTheta / Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_ANGULAR_RADIUS,
          lookaheadRadius)) {
        targetIndex = i;
        targetPoint = pathPoints.getJSONObject(i);
        break;
      }
    }
    double targetX = targetPoint.getDouble("x"), targetY = targetPoint.getDouble("y"),
        targetTheta = targetPoint.getDouble("angle");

    while (Math.abs(targetTheta - currentTheta) > Math.PI) {
      if (targetTheta - currentTheta > Math.PI) {
        targetTheta -= 2 * Math.PI;
      } else if (targetTheta - currentTheta < -Math.PI) {
        targetTheta += 2 * Math.PI;
      }
    }
    xPID.setSetPoint(targetX);
    yPID.setSetPoint(targetY);
    thetaPID.setSetPoint(targetTheta);

    xPID.updatePID(currentX);
    yPID.updatePID(currentY);
    thetaPID.updatePID(currentTheta);

    double xVelNoFF = xPID.getResult();
    double yVelNoFF = yPID.getResult();
    double thetaVelNoFF = -thetaPID.getResult();
    double f = (accurate ? Constants.Autonomous.ACCURATE_FOLLOWER_AUTONOMOUS_END_ACCURACY
        : Constants.Autonomous.FEED_FORWARD_MULTIPLIER);
    double feedForwardX = targetPoint.getDouble("x_velocity") * f;
    double feedForwardY = targetPoint.getDouble("y_velocity") * f;
    double feedForwardTheta = -targetPoint.getDouble("angular_velocity") * f;

    double finalX = xVelNoFF + feedForwardX;
    double finalY = yVelNoFF + feedForwardY;
    double finalTheta = thetaVelNoFF + feedForwardTheta;
    if (m_fieldSide == "blue") {
      finalX = -finalX;
      finalTheta = -finalTheta;
    }
    Number[] velocityArray = new Number[] {
        finalX,
        -finalY,
        finalTheta,
        targetIndex,
    };

    double velocityMag = Math
        .sqrt(Math.pow(targetPoint.getDouble("x_velocity"), 2) + Math.pow(targetPoint.getDouble("y_velocity"), 2));
    Logger.recordOutput("x-vel", velocityArray[0].doubleValue());
    Logger.recordOutput("y-vel", velocityArray[1].doubleValue());
    Logger.recordOutput("theta-vel", velocityArray[2].doubleValue());
    Logger.recordOutput("wanted-theta-vel", targetPoint.getDouble("angular_velocity"));
    Logger.recordOutput("pid-theta-vel", thetaVelNoFF);
    Logger.recordOutput("FF-theta-vel", feedForwardTheta);
    Logger.recordOutput("current point idx", currentIndex);
    Logger.recordOutput("point idx", velocityArray[3].intValue());
    Logger.recordOutput("look-ahead", Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_DISTANCE * velocityMag + 0.01);
    return velocityArray;
  }

  public boolean insideRadius(double deltaX, double deltaY, double deltaTheta, double radius) {
    Logger.recordOutput("deltax", deltaX);
    Logger.recordOutput("deltay", deltaY);
    Logger.recordOutput("deltaTheta", deltaTheta);
    return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2) + Math.pow(deltaTheta, 2)) < radius;
  }

  public void calculateAngleChange(double angle) {
    double pigeonAngleDegrees = this.peripherals.getPigeonAngle();
    double targetAngle = 0;
    if (getFieldSide() == "red") {
      targetAngle = angle + 180;
    } else {
      targetAngle = angle;
    }

    if (DriverStation.isAutonomousEnabled() && getFieldSide() == "red") {
      pigeonAngleDegrees = 180 + pigeonAngleDegrees;
    }
    this.turningPID.setSetPoint(Constants.standardizeAngleDegrees(targetAngle));
    this.turningPID.updatePID(Constants.standardizeAngleDegrees(pigeonAngleDegrees));
    double turnResult = -turningPID.getResult();

    this.driveAutoAligned(turnResult);
  }

  private DriveState handleStateTransition() {
    switch (wantedState) {
      case DEFAULT:
        return DriveState.DEFAULT;
      case IDLE:
        return DriveState.IDLE;
      default:
        return DriveState.IDLE;
    }
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Robot Odometry", getMT2Odometry());
    updateOdometryFusedArray();
    // process inputs
    DriveState newState = handleStateTransition();
    if (newState != systemState) {
      systemState = newState;
    }
    Logger.recordOutput("Drive State", systemState);
    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      systemState = DriveState.DEFAULT;
    }
    switch (systemState) {
      case DEFAULT:
        teleopDrive();
        break;
      default:
    }
  }

  /**
   * Calculates the adjusted y-coordinate based on the original x and y
   * coordinates.
   *
   * @param originalX The original x-coordinate.
   * @param originalY The original y-coordinate.
   * @return The adjusted y-coordinate.
   */
  public double getAdjustedY(double originalX, double originalY) {
    double adjustedY = originalY * Math.sqrt((1 - (Math.pow(originalX, 2)) / 2));
    return adjustedY;
  }

  /**
   * Calculates the adjusted x-coordinate based on the original x and y
   * coordinates.
   *
   * @param originalX The original x-coordinate.
   * @param originalY The original y-coordinate.
   * @return The adjusted x-coordinate.
   */
  public double getAdjustedX(double originalX, double originalY) {
    double adjustedX = originalX * Math.sqrt((1 - (Math.pow(originalY, 2)) / 2));
    return adjustedX;
  }
}