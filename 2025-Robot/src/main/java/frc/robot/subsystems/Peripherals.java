package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.tools.math.Vector;

public class Peripherals {
  private PhotonCamera frontCam = new PhotonCamera("Front_Cam");
  private PhotonCamera gamePieceCamera = new PhotonCamera("Game_Piece_Camera");

  AprilTagFieldLayout aprilTagFieldLayout;

  private Pigeon2 pigeon = new Pigeon2(0, "Canivore");
  private Pigeon2Configuration pigeonConfig = new Pigeon2Configuration();
  Transform3d robotToCam = new Transform3d(
      new Translation3d(Constants.inchesToMeters(1.75), Constants.inchesToMeters(11.625),
          Constants.inchesToMeters(33.5)),
      new Rotation3d(0, Math.toRadians(30.6), 0));
  PhotonPoseEstimator photonPoseEstimator;

  double pigeonSetpoint = 0.0;

  public Peripherals() {
  }

  /**
   * Initializes the Peripherals subsystem.
   * 
   * This method sets up the IMU configuration, mount pose, and zeroes the IMU.
   * It also applies the default command to the Peripherals subsystem.
   */
  public void init() {
    try {
      aprilTagFieldLayout = new AprilTagFieldLayout(
          Filesystem.getDeployDirectory().getPath() + "/" + "2025-reefscape.json");
    } catch (Exception e) {
      System.out.println("error with april tag: " + e.getMessage());
    }
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
    // Set the mount pose configuration for the IMU
    pigeonConfig.MountPose.MountPosePitch = 0.3561480641365051;
    pigeonConfig.MountPose.MountPoseRoll = -0.10366992652416229;
    pigeonConfig.MountPose.MountPoseYaw = -0.24523599445819855;

    // Apply the IMU configuration
    pigeon.getConfigurator().apply(pigeonConfig);

    // Zero the IMU angle
    zeroPigeon();
  }

  public double getFrontCamYaw() {
    double yaw = 0.0;
    var result = frontCam.getLatestResult();
    Logger.recordOutput("has target", result.hasTargets());
    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      yaw = target.getYaw();
    }
    return yaw;
  }

  public double getFrontCamPitch() {
    double pitch = 0.0;
    var result = frontCam.getLatestResult();
    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      pitch = target.getPitch();
    }

    return pitch;
  }

  public double getGamePieceCamYaw() {
    double yaw = 0.0;
    var result = gamePieceCamera.getLatestResult();
    Logger.recordOutput("has target", result.hasTargets());
    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      yaw = target.getYaw();
    }
    return yaw;
  }

  public List<TargetCorner> getGamePieceCamCorners() {
    List<TargetCorner> corners = new ArrayList<TargetCorner>();
    var result = gamePieceCamera.getLatestResult();
    Logger.recordOutput("has target", result.hasTargets());
    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      corners = target.getDetectedCorners();
      TargetCorner corner = corners.get(0);
    }
    return corners;
  }

  public double getGamePiecePitch() {
    double pitch = 0.0;
    var result = gamePieceCamera.getLatestResult();
    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      pitch = target.getPitch();
    }

    return pitch;
  }

  private Pose2d getRobotPoseViaTrig(PhotonTrackedTarget trackedTarget, double[] cameraPositionOnRobot,
      double robotAngle) {
    double pitch = trackedTarget.getPitch();
    double yaw = trackedTarget.getYaw();
    int id = trackedTarget.getFiducialId();
    double cameraXOffset = cameraPositionOnRobot[0];
    double cameraYOffset = cameraPositionOnRobot[1];
    double cameraZOffset = cameraPositionOnRobot[2];
    double cameraRYOffset = cameraPositionOnRobot[4];
    double cameraRZOffset = cameraPositionOnRobot[5];

    double[] tagPose = Constants.Vision.TAG_POSES[id - 1];
    double tagHeight = tagPose[2];
    double tagX = tagPose[0];
    double tagY = tagPose[1];
    double tagYaw = tagPose[3];

    double distToTag = (tagHeight - cameraZOffset) / Math.tan(Math.toRadians(pitch + cameraRYOffset));
    Logger.recordOutput("Distance to Tag", distToTag);
    Logger.recordOutput("yaw to Tag", yaw);
    double txProjOntoGroundPlane = Math.atan((Math.tan(yaw)) / Math.cos(pitch));
    double xFromTag = distToTag * Math.cos(Math.toRadians(txProjOntoGroundPlane + robotAngle + cameraRZOffset));
    double yFromTag = distToTag * Math.sin(Math.toRadians(txProjOntoGroundPlane + robotAngle + cameraRZOffset));
    Logger.recordOutput("x to Tag", xFromTag);
    Logger.recordOutput("y to Tag", yFromTag);

    double fieldPoseX = -xFromTag + tagX - cameraXOffset;
    double fieldPoseY = -yFromTag + tagY - cameraYOffset;
    Pose2d pose = new Pose2d(fieldPoseX, fieldPoseY, new Rotation2d(Math.toRadians(getPigeonAngle())));
    return pose;
  }

  /**
   * Calculates the robot's position based on the camera offset, angles to the
   * tag, and tag position.
   *
   * @param cameraOffset    Pose3d representing the camera's position and
   *                        orientation relative to the robot center.
   * @param horizontalAngle Horizontal angle (radians) from the camera to the tag.
   * @param verticalAngle   Vertical angle (radians) from the camera to the tag.
   * @param tagPosition     Pose3d representing the position of the tag in the
   *                        field coordinate system.
   * @param robotYaw        Yaw (rotation around the Z-axis) of the robot in
   *                        radians.
   * @return Pose3d representing the robot's position in the field coordinate
   *         system.
   */
  public static Pose3d calculateRobotPosition(
      Pose3d cameraOffset, double horizontalAngle, double verticalAngle, Pose3d tagPosition, double robotYaw) {

    // Extract the camera's offset from the robot in its local frame
    Translation3d cameraTranslation = cameraOffset.getTranslation();

    // Rotate the camera's offset into the field frame using the robot's yaw
    double cosYaw = Math.cos(robotYaw);
    double sinYaw = Math.sin(robotYaw);
    Translation3d cameraInFieldTranslation = new Translation3d(
        cosYaw * cameraTranslation.getX() - sinYaw * cameraTranslation.getY(),
        sinYaw * cameraTranslation.getX() + cosYaw * cameraTranslation.getY(),
        cameraTranslation.getZ());

    // Calculate the relative position of the tag to the camera
    double dx = tagPosition.getTranslation().getX() - cameraInFieldTranslation.getX();
    double dy = tagPosition.getTranslation().getY() - cameraInFieldTranslation.getY();
    double dz = tagPosition.getTranslation().getZ() - cameraInFieldTranslation.getZ();

    // Calculate the distance from the camera to the tag
    double distanceToTag = Math.sqrt(dx * dx + dy * dy + dz * dz);

    // Translation from the camera to the tag in the camera's local space
    Translation3d tagInCameraSpace = new Translation3d(
        distanceToTag * Math.cos(horizontalAngle) * Math.cos(verticalAngle), // X
        distanceToTag * Math.sin(horizontalAngle) * Math.cos(verticalAngle), // Y
        distanceToTag * Math.sin(verticalAngle) // Z
    );

    // Transform the tag position from the camera space to the field space
    Translation3d tagInFieldSpace = cameraInFieldTranslation.plus(tagInCameraSpace);

    // Calculate the robot's position in the field frame
    Translation3d robotTranslation = tagPosition.getTranslation().minus(tagInFieldSpace);

    // Return the robot's position with its yaw in the field coordinate system
    return new Pose3d(robotTranslation, new Rotation3d(0, 0, robotYaw));
  }

  public Pose2d getFrontCamTrigPose() {
    var result = frontCam.getLatestResult();
    if (result.hasTargets() && result.getBestTarget().getPoseAmbiguity() < 0.3) {
      PhotonTrackedTarget target = result.getBestTarget();
      // Pose3d robotPose = calculateRobotPosition(cameraOffset, target.getYaw(),
      // target.getPitch(),
      // new Pose3d(
      // new Translation3d(Constants.Vision.TAG_POSES[10][0],
      // Constants.Vision.TAG_POSES[10][1],
      // Constants.Vision.TAG_POSES[10][2]),
      // new Rotation3d(0.0, Constants.Vision.TAG_POSES[10][4],
      // Constants.Vision.TAG_POSES[10][3])),
      // Math.toRadians(getPigeonAngle()));
      Pose2d robotPose = getRobotPoseViaTrig(target,
          Constants.Vision.FRONT_CAMERA_POSE, getPigeonAngle());
      Logger.recordOutput("Trig Localiazation", robotPose);
      return robotPose;
    } else {
      Pose2d defaultPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
      return defaultPose;
    }
  }

  public Pose3d getFrontCamPnPPose() {
    // Logger.recordOutput("April Tag", aprilTagFieldLayout.getTagPose(1).get());
    var result = frontCam.getLatestResult();
    Optional<EstimatedRobotPose> multiTagResult = photonPoseEstimator.update(result);
    if (multiTagResult.isPresent()) {
      Pose3d robotPose = multiTagResult.get().estimatedPose;
      Logger.recordOutput("multitag result", robotPose);
      return robotPose;
    } else {
      Pose3d robotPose = new Pose3d();
      return robotPose;
    }
    // var result = frontCam.getLatestResult();
    // Logger.recordOutput("Is presetn", result.getMultiTagResult().isPresent());
    // if (result.getMultiTagResult().isPresent()) {
    // Transform3d fieldToCamera =
    // result.getMultiTagResult().get().estimatedPose.best;
    // Logger.recordOutput("Camera pose: ", fieldToCamera);
    // return fieldToCamera;
    // } else {
    // System.out.println("in else");
    // return new Transform3d();
    // }
  }

  public double getFrontCamLatency() {
    return frontCam.getLatestResult().getTimestampSeconds();
  }

  /**
   * Sets the IMU angle to 0
   */
  public void zeroPigeon() {
    setPigeonAngle(0.0);
  }

  /**
   * Sets the angle of the IMU
   * 
   * @param degrees - Angle to be set to the IMU
   */
  public void setPigeonAngle(double degrees) {
    pigeon.setYaw(degrees);
  }

  /**
   * Retrieves the yaw of the robot
   * 
   * @return Yaw in degrees
   */
  public double getPigeonAngle() {
    return pigeon.getYaw().getValueAsDouble();
  }

  /**
   * Retrieves the absolute angular velocity of the IMU's Z-axis in device
   * coordinates.
   *
   * @return The absolute angular velocity of the IMU's Z-axis in device
   *         coordinates.
   *         The value is in degrees per second.
   */
  public double getPigeonAngularVelocity() {
    return Math.abs(pigeon.getAngularVelocityZDevice().getValueAsDouble());
  }

  /**
   * Retrieves the absolute angular velocity of the IMU's Z-axis in world
   * coordinates.
   *
   * @return The absolute angular velocity of the IMU's Z-axis in world
   *         coordinates.
   *         The value is in radians per second.
   */
  public double getPigeonAngularVelocityW() {
    return pigeon.getAngularVelocityZWorld().getValueAsDouble();
  }

  /**
   * Retrieves the acceleration vector of the robot
   * 
   * @return Current acceleration vector of the robot
   */
  public Vector getPigeonLinAccel() {
    Vector accelVector = new Vector();
    accelVector.setI(pigeon.getAccelerationX().getValueAsDouble() / Constants.Physical.GRAVITY_ACCEL_MS2);
    accelVector.setJ(pigeon.getAccelerationY().getValueAsDouble() / Constants.Physical.GRAVITY_ACCEL_MS2);
    return accelVector;
  }

  public void periodic() {
    // getFrontCamPnPPose(); //TODO: uncomment when using camera
    // var result = frontCam.getLatestResult();
    // if (result.hasTargets()) {
    // PhotonTrackedTarget target = result.getBestTarget();
    // Pose2d robotPose = getRobotPoseViaTrig(target,
    // Constants.Vision.FRONT_CAMERA_POSE, getPigeonAngle());
    // Logger.recordOutput("Trig Localiazation", robotPose);
    // }
  }
}