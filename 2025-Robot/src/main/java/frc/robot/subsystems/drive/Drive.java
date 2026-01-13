package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;
import org.json.JSONArray;
import org.json.JSONObject;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.tools.controlloops.PID;
import frc.robot.tools.math.Vector;

// **Zero Wheels with the bolt head showing on the left when the front side(battery) is facing down/away from you**

public class Drive extends SubsystemBase {

  DriveIO io;
  Peripherals peripherals;
  boolean firstClimb = false;

  // odometry

  private double m_initTime;
  private double m_currentTime;

  // Creating my kinematics object using the module locations

  double initAngle;
  double setAngle;
  double diffAngle;

  // path following PID values
  private double kXP = 4.00;
  private double kXI = 0.00;
  private double kXD = 1.20;

  private double kYP = kXP;
  private double kYI = kXI;
  private double kYD = kXD;

  private double kThetaP = 2.90;
  private double kThetaI = 0.00;
  private double kThetaD = 2.00;

  // auto placement PID values
  private double kkXP = 5.50;
  private double kkXI = 0.00;
  private double kkXD = 1.60;

  private double kkYP = kkXP;
  private double kkYI = kkXI;
  private double kkYD = kkXD;

  private double kkThetaP = 1.79;
  private double kkThetaI = 0.00;
  private double kkThetaD = 0.971;

  // l4 pid values
  private double kkXP4 = 4.30;
  private double kkXI4 = 0.00;
  private double kkXD4 = 1.70;

  private double kkYP4 = kkXP4;
  private double kkYI4 = kkXI4;
  private double kkYD4 = kkXD4;

  private double kkThetaP4 = 2.056;
  private double kkThetaI4 = 0.00;
  private double kkThetaD4 = 0.971;

  // l4 auto pids
  private double scalar = 0.9;
  private double kkXP4A = 4.30 * scalar;
  private double kkXI4A = 0.00 * scalar;
  private double kkXD4A = 1.70 * scalar;

  private double kkYP4A = kkXP4A;
  private double kkYI4A = kkXI4A;
  private double kkYD4A = kkXD4A;

  private double kkThetaP4A = 2.056;
  private double kkThetaI4A = 0.00;
  private double kkThetaD4A = 0.971;

  // l23 pid values
  private double kkXP23 = 4.00;
  private double kkXI23 = 0.00;
  private double kkXD23 = 1.60;

  private double kkYP23 = kkXP23;
  private double kkYI23 = kkXI23;
  private double kkYD23 = kkXD23;

  private double kkThetaP23 = 2.481;
  private double kkThetaI23 = 0.00;
  private double kkThetaD23 = 0.971;

  // l1 pid values
  private double kkXP1 = 4.00;
  private double kkXI1 = 0.00;
  private double kkXD1 = 1.40;

  private double kkYP1 = kkXP1;
  private double kkYI1 = kkXI1;
  private double kkYD1 = kkXD1;

  private double kkThetaP1 = 3.005;
  private double kkThetaI1 = 0.00;
  private double kkThetaD1 = 0.971;

  // Piece pickup values
  private double kkkXPPickup = 3.30;
  private double kkkXIPickup = 0.00;
  private double kkkXDPickup = 1.70;

  private double kkkYPPickup = kkkXPPickup;
  private double kkkYIPickup = kkkXIPickup;
  private double kkkYDPickup = kkkXDPickup;

  private double kkkThetaPPickup = 2.056;
  private double kkkThetaIPickup = 0.00;
  private double kkkThetaDPickup = 0.971;

  // teleop targeting PID values
  private double kTurningP = 0.04;
  private double kTurningI = 0;
  private double kTurningD = 0.06;
  private double kRotateP = 0.04;
  private double kRotateI = 0.0;
  private double kRotateD = 0.06;

  private PID xxPID = new PID(kkXP, kkXI, kkXD);
  private PID yyPID = new PID(kkYP, kkYI, kkYD);
  private PID thetaaPID = new PID(kkThetaP, kkThetaI, kkThetaD);

  private PID xxPID4 = new PID(kkXP4, kkXI4, kkXD4);
  private PID yyPID4 = new PID(kkYP4, kkYI4, kkYD4);
  private PID thetaaPID4 = new PID(kkThetaP4, kkThetaI4, kkThetaD4);

  private PID xxPID4A = new PID(kkXP4A, kkXI4A, kkXD4A);
  private PID yyPID4A = new PID(kkYP4A, kkYI4A, kkYD4A);
  private PID thetaaPID4A = new PID(kkThetaP4A, kkThetaI4A, kkThetaD4A);

  private PID xxPID23 = new PID(kkXP23, kkXI23, kkXD23);
  private PID yyPID23 = new PID(kkYP23, kkYI23, kkYD23);
  private PID thetaaPID23 = new PID(kkThetaP23, kkThetaI23, kkThetaD23);

  private PID xxPID1 = new PID(kkXP1, kkXI1, kkXD1);
  private PID yyPID1 = new PID(kkYP1, kkYI1, kkYD1);
  private PID thetaaPID1 = new PID(kkThetaP1, kkThetaI1, kkThetaD1);

  private PID xxPIDPickup = new PID(kkkXPPickup, kkkXIPickup, kkkXDPickup);
  private PID yyPIDPickup = new PID(kkkYPPickup, kkkYIPickup, kkkYDPickup);
  private PID thetaaPIDPickup = new PID(kkkThetaPPickup, kkkThetaIPickup, kkkThetaDPickup);

  private PID xPID = new PID(kXP, kXI, kXD);
  private PID yPID = new PID(kYP, kYI, kYD);
  private PID thetaPID = new PID(kThetaP, kThetaI, kThetaD);
  private PID turningPID = new PID(kTurningP, kTurningI, kTurningD);
  private PID rotatePID = new PID(kRotateP, kRotateI, kRotateD);

  private String m_fieldSide = "blue";

  AprilTagFieldLayout aprilTagFieldLayout;
  double startX;
  double startY;

  public boolean algaeMode = false;

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
    if (RobotBase.isReal()) {
      this.io = new DriveIOComp(this.peripherals);
    } else {
      this.io = new DriveIOSim();
    }
  }

  // public boolean atSetpoint() {
  // double currentAngle = gyro.getYawDegrees();
  // if (getFieldSide().equals("red")) {
  // currentAngle -= 180;
  // }
  // return (Math.abs(Constants.standardizeAngleDegrees(currentAngle)
  // - Constants.standardizeAngleDegrees(angleSetpoint)) < 2);
  // }

  public void setWantedState(DriveState wantedState) {
    this.wantedState = wantedState;
  }

  public void setWantedState(DriveState wantedState, double angle) {
    this.wantedState = wantedState;
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

    try {
      aprilTagFieldLayout = new AprilTagFieldLayout(
          Filesystem.getDeployDirectory().getPath() + "/" + "2025-reefscape-andymark.json");
    } catch (Exception e) {
    }
    xxPID.setMinOutput(-3.0);
    xxPID.setMaxOutput(3.0);

    yyPID.setMinOutput(-3.0);
    yyPID.setMaxOutput(3.0);

    thetaaPID.setMinOutput(-3.0);
    thetaaPID.setMaxOutput(3.0);

    xxPID4.setMinOutput(-2.0);
    xxPID4.setMaxOutput(2.0);

    yyPID4.setMinOutput(-2.0);
    yyPID4.setMaxOutput(2.0);

    thetaaPID4.setMinOutput(-2.0);
    thetaaPID4.setMaxOutput(2.0);

    xxPID4A.setMinOutput(-1.4);
    xxPID4A.setMaxOutput(1.4);

    yyPID4A.setMinOutput(-1.4);
    yyPID4A.setMaxOutput(1.4);

    thetaaPID4A.setMinOutput(-1.4);
    thetaaPID4A.setMaxOutput(1.4);

    xxPID23.setMinOutput(-4.0);
    xxPID23.setMaxOutput(4.0);

    yyPID23.setMinOutput(-4.0);
    yyPID23.setMaxOutput(4.0);

    thetaaPID23.setMinOutput(-4.0);
    thetaaPID23.setMaxOutput(4.0);

    xxPID1.setMinOutput(-4.0);
    xxPID1.setMaxOutput(4.0);

    yyPID1.setMinOutput(-4.0);
    yyPID1.setMaxOutput(4.0);

    thetaaPID1.setMinOutput(-4.0);
    thetaaPID1.setMaxOutput(4.0);

    xxPIDPickup.setMinOutput(-1.0);
    xxPIDPickup.setMaxOutput(1.0);

    yyPIDPickup.setMinOutput(-1.0);
    yyPIDPickup.setMaxOutput(1.0);

    thetaaPIDPickup.setMinOutput(-2.0);
    thetaaPIDPickup.setMaxOutput(2.0);

    xPID.setMinOutput(-4.9);
    xPID.setMaxOutput(4.9);

    yPID.setMinOutput(-4.9);
    yPID.setMaxOutput(4.9);

    thetaPID.setMinOutput(-3);
    thetaPID.setMaxOutput(3);

    turningPID.setMinOutput(-3);
    turningPID.setMaxOutput(3);

    rotatePID.setMinOutput(-2);
    rotatePID.setMaxOutput(2);
  }

  public void teleopInit() {

  }

  /**
   * Zeros the IMU (Inertial Measurement Unit) mid-match and resets the odometry
   * with a zeroed angle.
   * It resets the angle reported by the pigeon sensor to zero and updates the
   * odometry with this new zeroed angle.
   */
  public void zeroIMU() {
    io.zeroIMU();
  }

  /**
   * Adjusts the angle reported by the pigeon sensor after an autonomous routine.
   * It adds 180 degrees to the current angle reported by the pigeon sensor and
   * wraps it around 360 degrees.
   */
  public void setPigeonAfterAuto() {
    io.setYaw((io.getYaw().getDegrees() + 180) % 360);
  }

  /**
   * Sets the angle reported by the pigeon sensor to the specified value.
   *
   * @param angle The angle to set for the pigeon sensor in degrees.
   */
  public void setPigeonAngle(double angle) {
    io.setYaw(angle);
  }

  /**
   * Retrieves the current angle reported by the pigeon sensor.
   *
   * @return The current angle reported by the pigeon sensor in degrees.
   */
  public double getPigeonAngle() {
    return io.getYaw().getDegrees();
  }

  /**
   * Sets the PID values for all swerve modules to zero, keeping the wheels
   * straight.
   */
  public void setWheelsStraight() {

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
    JSONObject firstPoint = pathPoints.getJSONObject(0);
    double firstPointX = firstPoint.getDouble("x");
    double firstPointY = firstPoint.getDouble("y");
    double firstPointAngle = firstPoint.getDouble("angle");

    // changing odometry if on red side, don't need to change y because it will be
    // the same for autos on either side
    if (this.m_fieldSide == "blue") {
      firstPointX = Constants.Physical.FIELD_LENGTH - firstPointX;
      firstPointY = Constants.Physical.FIELD_WIDTH - firstPointY;
      firstPointAngle = Math.PI + firstPointAngle;
    }

    if (OI.isProcessorSide()) {
      firstPointY = Constants.Physical.FIELD_WIDTH - firstPointY;
      firstPointAngle = -firstPointAngle;
    }
    Pose2d firstPose2d = new Pose2d(new Translation2d(firstPointX, firstPointY), new Rotation2d(firstPointAngle));
    io.setCurrentLimits(60, 120);
    io.setPosition(firstPose2d);

    m_initTime = Timer.getFPGATimestamp();
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

  public void setOdometry(Pose2d pose) {
    io.setPosition(pose);
  }

  public boolean isPoseInField(Pose2d pose) {
    if (pose.getY() < 0 || pose.getY() > Constants.Physical.FIELD_WIDTH || pose.getX() < 0
        || pose.getX() > Constants.Physical.FIELD_LENGTH) {
      return false;
    } else {
      return true;
    }
  }

  private boolean autoPlacingFront = true;

  public double getAngleDifferenceDegrees(double angle1, double angle2) {
    double difference = Math.abs(angle1 - angle2) % 360;
    return difference > 180 ? 360 - difference : difference;
  }

  boolean firstTimeAutoPickup = false;
  double firstTimePickupAngle = 0.0;
  boolean firstTimeCalculated = false;
  boolean firstTimeGoingInCalculated = false;
  boolean firstTimeGoingIn = false;
  boolean hasTrack = false;

  Pose2d targetPose = new Pose2d();
  Pose2d c1 = new Pose2d();
  Pose2d c2 = new Pose2d();

  public double getGyroYaw() {
    return io.getYaw().getDegrees();
  }

  public Pose2d getMt2Pose2d() {
    return io.getPosition();
  }

  /**
   * Retrieves the current X-coordinate of the robot from odometry.
   *
   * @return The current X-coordinate of the robot.
   */
  public double getMt2Pose2dX() {
    return getMt2Pose2d().getX();
  }

  /**
   * Retrieves the current Y-coordinate of the robot from odometry.
   *
   * @return The current Y-coordinate of the robot.
   */
  public double getMt2Pose2dY() {
    return getMt2Pose2d().getY();
  }

  /**
   * Retrieves the current orientation angle of the robot from odometry.
   *
   * @return The current orientation angle of the robot in radians.
   */
  public double getMt2Pose2dAngle() {
    return getMt2Pose2d().getRotation().getRadians();
  }

  /**
   * Drives the robot with alignment adjustment based on the specified angle from
   * placement.
   * 
   * @param degreesFromPlacement The angle in degrees from the placement
   *                             orientation to align with.
   */
  public void driveAutoAligned(double degreesFromPlacement) {

    double turn = degreesFromPlacement;

    double originalX = -(Math.copySign(OI.getDriverLeftY() * OI.getDriverLeftY(), OI.getDriverLeftY()));
    double originalY = -(Math.copySign(OI.getDriverLeftX() * OI.getDriverLeftX(), OI.getDriverLeftX()));

    if (Math.abs(originalX) < 0.05) {
      originalX = 0;
    }
    if (Math.abs(originalY) < 0.05) {
      originalY = 0;
    }

    double xPower = getAdjustedX(originalX, originalY);
    double yPower = getAdjustedY(originalX, originalY);

    double xSpeed = xPower * Constants.Physical.TOP_SPEED;
    double ySpeed = yPower * Constants.Physical.TOP_SPEED;

    Vector controllerVector = new Vector(xSpeed, ySpeed);
    if (getFieldSide().equals("red")) {
      controllerVector.setI(-xSpeed);
      controllerVector.setJ(-ySpeed);
    }
    io.drive(controllerVector, turn);
  }

  /**
   * Turns the robot in robot-centric mode.
   * 
   * @param turn The rate at which the robot should turn in radians per second.
   */
  public void autoRobotCentricTurn(double turn) {
    io.drive(new Vector(0, 0), turn);
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
    io.driveRobotCentric(velocityVector, turnRadiansPerSec);
  }

  /**
   * Drives the robot during teleoperation.
   * 
   * @apiNote This method updates the fused odometry array and controls the
   *          robot's movement based on joystick inputs.
   */
  public void teleopDrive() {
    double oiRX = OI.getDriverRightX();
    double oiLX = OI.getDriverLeftX();
    double oiRY = OI.getDriverRightY();
    double oiLY = OI.getDriverLeftY();
    if (OI.operatorLT.getAsBoolean() && OI.operatorRT.getAsBoolean()) {
      oiRX = OI.getOperatorRightX();
      oiLX = OI.getOperatorLeftX();
      oiRY = OI.getOperatorRightY();
      oiLY = OI.getOperatorLeftY();
    }

    double turnLimit = 0.17;

    if (OI.driverController.getRightTriggerAxis() > 0.2 || OI.getDriverRB()) {
      // activate slowy spin
      turnLimit = 0.1;
      oiRX = oiRX * 0.8;
      oiLX = oiLX * 0.8;
      oiRY = oiRY * 0.8;
      oiLY = oiLY * 0.8;
    }
    double originalX = -(Math.copySign(oiLY * oiLY, oiLY));
    double originalY = -(Math.copySign(oiLX * oiLX, oiLX));
    double turn = turnLimit
        * (oiRX * (Constants.Physical.TOP_SPEED) / (Constants.Physical.ROBOT_RADIUS));

    if (Math.abs(turn) < 0.05) {
      turn = 0.0;
    }
    double xPower = getAdjustedX(originalX, originalY);
    double yPower = getAdjustedY(originalX, originalY);

    double xSpeed = xPower * Constants.Physical.TOP_SPEED;
    double ySpeed = yPower * Constants.Physical.TOP_SPEED;

    Vector controllerVector = new Vector(xSpeed, ySpeed);
    if (getFieldSide().equals("red")) {
      controllerVector.setI(-xSpeed);
      controllerVector.setJ(-ySpeed);
    }
    io.drive(controllerVector, turn);
  }

  public void robotCentricDrive(double angle) {
    double oiRX = OI.getDriverRightX();
    double oiLX = OI.getDriverLeftX();
    double oiRY = OI.getDriverRightY();
    double oiLY = OI.getDriverLeftY();

    double turnLimit = 0.17;
    // 0.35 before

    if (OI.driverController.getRightTriggerAxis() > 0.2) {
      // activate slowy spin
      turnLimit = 0.1;
      oiRX = oiRX * 0.5;
      oiLX = oiLX * 0.5;
      oiRY = oiRY * 0.5;
      oiLY = oiLY * 0.5;
    }

    // this is correct, X is forward in field, so originalX should be the y on the
    // // joystick
    // double originalX = -(Math.copySign(OI.getDriverLeftY() * OI.getDriverLeftY(),
    // OI.getDriverLeftY()));
    // double originalY = -(Math.copySign(OI.getDriverLeftX() * OI.getDriverLeftX(),
    // OI.getDriverLeftX()));
    double originalX = -(Math.copySign(oiLY * oiLY, oiLY));
    double originalY = -(Math.copySign(oiLX * oiLX, oiLX));
    // if (Math.abs(originalX) < 0.005) {
    // originalX = 0;
    // }
    // if (Math.abs(originalY) < 0.005) {
    // originalY = 0;
    // }

    // double turn = turnLimit * ((Math.copySign(OI.getDriverRightX() *
    // OI.getDriverRightX() * OI.getDriverRightX(), OI.getDriverRightX())) *
    // (Constants.Physical.TOP_SPEED)/(Constants.Physical.ROBOT_RADIUS));
    double turn = turnLimit
        * (oiRX * (Constants.Physical.TOP_SPEED) / (Constants.Physical.ROBOT_RADIUS));

    if (Math.abs(turn) < 0.05) {
      turn = 0.0;
    }

    double xPower = getAdjustedX(originalX, originalY);
    double yPower = getAdjustedY(originalX, originalY);

    double xSpeed = xPower * Constants.Physical.TOP_SPEED;
    double ySpeed = yPower * Constants.Physical.TOP_SPEED;

    Vector controllerVector = new Vector(xSpeed, ySpeed);
    if (getFieldSide().equals("red")) {
      controllerVector.setI(-xSpeed);
      controllerVector.setJ(-ySpeed);
    }
    io.driveCamCentric(controllerVector, turn, Math.toRadians(angle));
  }

  public void driveToPoint(Pose2d targetPoint) {

    double x = targetPoint.getX();
    double y = targetPoint.getY();
    double theta = targetPoint.getRotation().getRadians();
    theta = Constants.standardizeAngleToOther(theta, getMt2Pose2dAngle());

    double xVelNoFF = 0.0;
    double yVelNoFF = 0.0;
    double thetaVelNoFF = 0.0;

    if (OI.driverPOVRight.getAsBoolean()) {
      xxPID4.setSetPoint(x);
      yyPID4.setSetPoint(y);
      thetaaPID4.setSetPoint(theta);

      xxPID4.updatePID(getMt2Pose2dX());
      yyPID4.updatePID(getMt2Pose2dY());
      thetaaPID4.updatePID(getMt2Pose2dAngle());

      xVelNoFF = xxPID4.getResult();
      yVelNoFF = yyPID4.getResult();
      thetaVelNoFF = -thetaaPID4.getResult();

    } else if (DriverStation.isTeleopEnabled()
        && (OI.driverPOVLeft.getAsBoolean() || OI.driverPOVDown.getAsBoolean())) {

      xxPID23.setSetPoint(x);
      yyPID23.setSetPoint(y);
      thetaaPID23.setSetPoint(theta);

      xxPID23.updatePID(getMt2Pose2dX());
      yyPID23.updatePID(getMt2Pose2dY());
      thetaaPID23.updatePID(getMt2Pose2dAngle());

      xVelNoFF = xxPID23.getResult();
      yVelNoFF = yyPID23.getResult();
      thetaVelNoFF = -thetaaPID23.getResult();

    } else if (DriverStation.isTeleopEnabled() && OI.driverPOVUp.getAsBoolean()) {

      xxPID1.setSetPoint(x);
      yyPID1.setSetPoint(y);
      thetaaPID1.setSetPoint(theta);

      xxPID1.updatePID(getMt2Pose2dX());
      yyPID1.updatePID(getMt2Pose2dY());
      thetaaPID1.updatePID(getMt2Pose2dAngle());

      xVelNoFF = xxPID1.getResult();
      yVelNoFF = yyPID1.getResult();
      thetaVelNoFF = -thetaaPID1.getResult();

    } else {

      xxPID.setSetPoint(x);
      yyPID.setSetPoint(y);
      thetaaPID.setSetPoint(theta);

      xxPID.updatePID(getMt2Pose2dX());
      yyPID.updatePID(getMt2Pose2dY());
      thetaaPID.updatePID(getMt2Pose2dAngle());

      xVelNoFF = xxPID.getResult();
      yVelNoFF = yyPID.getResult();
      thetaVelNoFF = -thetaaPID.getResult();
    }

    // double feedForwardX = targetPoint.getDouble("x_velocity") *
    // Constants.Autonomous.FEED_FORWARD_MULTIPLIER;
    // double feedForwardY = targetPoint.getDouble("y_velocity") *
    // Constants.Autonomous.FEED_FORWARD_MULTIPLIER;
    // double feedForwardTheta = -targetPoint.getDouble("angular_velocity") *
    // Constants.Autonomous.FEED_FORWARD_MULTIPLIER;

    double finalX = xVelNoFF;
    double finalY = yVelNoFF;
    double finalTheta = thetaVelNoFF;
    // if (m_fieldSide == "blue") {
    // finalX = -finalX;
    // finalTheta = -finalTheta;
    // }
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

  public double getSpeedUsingPhysics(double distance, double finalVel) {
    // Max Velocity at which the robot can slow down given the Max Deceleration
    // (-Constants.Physical.MAX_ACCELERATION)
    return Math.sqrt(finalVel * finalVel - (2 * (-Constants.Physical.MAX_ACCELERATION) * distance));
  }

  public double clampToForwardAccelerationLimit(double currentVelocity, double wantedAcceleration) {
    return Math.min(wantedAcceleration,
        Constants.Physical.MAX_ACCELERATION * (1 - (currentVelocity / Constants.Physical.TOP_SPEED)));
  }

  public void driveToXTheta(double x, double theta) {
    // theta = Math.toRadians(theta);
    theta = Constants.standardizeAngleToOther(theta, getMt2Pose2dAngle());
    xxPID.setSetPoint(x);
    thetaaPID.setSetPoint(theta);

    xxPID.updatePID(getMt2Pose2dX());
    thetaaPID.updatePID(getMt2Pose2dAngle());

    double xVelNoFF = xxPID.getResult();
    double yVelNoFF = OI.getDriverLeftX() * 2.9;
    double thetaVelNoFF = -thetaaPID.getResult();
    double finalX = xVelNoFF;
    double finalY = yVelNoFF;
    double finalTheta = thetaVelNoFF;
    Number[] velocityArray = new Number[] {
        finalX,
        -finalY,
        finalTheta,
    };

    Vector velocityVector = new Vector();
    double desiredThetaChange = 0;
    if (getFieldSide().equals("red")) {
      velocityVector.setI(velocityArray[0].doubleValue());
      velocityVector.setJ(-velocityArray[1].doubleValue());
    } else {
      velocityVector.setI(velocityArray[0].doubleValue());
      velocityVector.setJ(velocityArray[1].doubleValue());
    }
    desiredThetaChange = velocityArray[2].doubleValue();

    autoDrive(velocityVector, desiredThetaChange);
  }

  public void driveToTheta(double theta) {
    theta = Constants.standardizeAngleToOtherDegrees(theta, getMt2Pose2dAngle());

    turningPID.setSetPoint(theta);
    turningPID.updatePID(Math.toDegrees(getMt2Pose2dAngle()));

    double result = -turningPID.getResult();
    if (Math.abs(Math.toDegrees(getMt2Pose2dAngle()) - theta) < 2) {
      result = 0;
    }
    driveAutoAligned(result);
  }

  /**
   * Runs autonomous driving by providing velocity vector and turning rate.
   * 
   * @param vector            The velocity vector containing xy velocities.
   * @param turnRadiansPerSec The rate at which the robot should spin in radians
   *                          per second.
   */
  public void autoDrive(Vector vector, double turnRadiansPerSec) {
    io.drive(vector, turnRadiansPerSec);
  }

  /**
   * Retrieves the current velocity vector of the robot in field coordinates.
   * The velocity vector is calculated based on the individual wheel speeds and
   * orientations.
   *
   * @return The current velocity vector of the robot in meters per second (m/s).
   */
  public Vector getRobotVelocityVector() {
    Vector velocityVector = io.getVelocityVector();
    return velocityVector;
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

  public boolean insideRadius(double deltaX, double deltaY, double deltaTheta, double radius) {

    return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2) + Math.pow(deltaTheta, 2)) < radius;
  }

  public void calculateAngleChange(double angle) {
    double pigeonAngleDegrees = io.getYaw().getDegrees();
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

  Vector scoreL23Vector = new Vector(2.5, 0);
  Pose2d l23Setpoint = new Pose2d();

  Vector pickupAlgaeFrontVector = new Vector(2.5, 0);
  Pose2d algaeSetpoint = new Pose2d();
  Vector pickupAlgaeBackVector = new Vector(-2.5, 0);

  public double getDistanceFromL23Setpoint() {
    return l23Setpoint.getTranslation().getDistance(getMt2Pose2d().getTranslation());
  }

  public double getDistanceFromAlgaeSetpoint() {
    return algaeSetpoint.getTranslation().getDistance(getMt2Pose2d().getTranslation());
  }

  public double[] getClosestPoint(double[] lineStart, double[] lineEnd) { // chatGPT ahh code
    // System.out.println("X1 - X2; " + (lineStart[0] - lineEnd[0]) + "Y1 - Y2" +
    // (lineStart[1] - lineEnd[1]));
    double x1 = lineStart[0];
    double y1 = lineStart[1];
    double x2 = lineEnd[0];
    double y2 = lineEnd[1];
    double px = getMt2Pose2dX();
    double py = getMt2Pose2dY();

    double dx = x2 - x1;
    double dy = y2 - y1;
    double lenSq = dx * dx + dy * dy;

    if (lenSq == 0) {
      return new double[] { x1, y1 };
    }

    double t = ((px - x1) * dx + (py - y1) * dy) / lenSq;
    t = Math.max(0, Math.min(1, t));

    double closestX = x1 + t * dx;
    double closestY = y1 + t * dy;

    return new double[] { closestX, closestY };
  }

  public double getThetaToPoint(double xMeters, double yMeters) {
    return Math.atan2(yMeters - getMt2Pose2dY(),
        xMeters - getMt2Pose2dX());
  }

  public Pose2d getClosestPose(Pose2d pose1, Pose2d pose2) {
    double dist1 = Math.hypot(Math.abs(pose1.getX() - getMt2Pose2dX()), Math.abs(pose1.getY() - getMt2Pose2dY()));
    double dist2 = Math.hypot(Math.abs(pose2.getX() - getMt2Pose2dX()), Math.abs(pose2.getY() - getMt2Pose2dY()));
    if (dist1 <= dist2) {
      return pose1;
    } else {
      return pose2;
    }
  }

  public Pose2d origionalSetpointPose = new Pose2d();
  public boolean firstTimeReef = true;

  public boolean isOnBlueSide() {
    return io.getPosition().getX() < Constants.Physical.FIELD_LENGTH / 2.0;
  }

  private final Vector backupVector = new Vector(-20.0, 0.0);
  private final Vector otherBackupVector = new Vector(20.0, 0.0);

  @Override
  public void periodic() {
    // Pose2d target = getGamePiecePosition();
    // System.out.println(Math.toDegrees(getThetaToCenterReef()));
    // Translation2d t1 = new Translation2d(getMt2Pose2dX(), getMt2Pose2dY());
    // Rotation2d r1 = new Rotation2d(getThetaToCenterReef());
    // Pose2d p1 = new Pose2d(t1, r1);
    io.update(systemState);
    // process inputs
    DriveState newState = handleStateTransition();
    Pose2d setpoint = new Pose2d();
    double standardizedAngle = Constants.standardizeAngleDegrees(Math.toDegrees(getMt2Pose2dAngle()));
    if (newState != systemState) {
      systemState = newState;
    }
    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      systemState = DriveState.DEFAULT;
    }

    if (!OI.getDriverA()) {
      firstTimeReef = true;
    }
    switch (systemState) {
      case DEFAULT:
        if (OI.driverA.getAsBoolean() && !(OI.driverPOVDown.getAsBoolean() || OI.driverPOVLeft.getAsBoolean()
            || OI.driverPOVUp.getAsBoolean() || OI.driverPOVRight.getAsBoolean())) {
          robotCentricDrive(195.0);
        } else {
          teleopDrive();
        }
        break;
      case IDLE:
        break;
      default:
        break;
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