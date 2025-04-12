package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.tools.math.Vector;

public class Peripherals {
  private PhotonCamera frontReefCam = new PhotonCamera("Front_Reef");
  private PhotonCamera backReefCam = new PhotonCamera("Back_Reef");
  private PhotonCamera frontBargeCam = new PhotonCamera("Front_Barge");
  private PhotonCamera backBargeCam = new PhotonCamera("Back_Barge");
  private PhotonCamera gamePieceCamera = new PhotonCamera("Front_Game_Piece_Cam");

  AprilTagFieldLayout aprilTagFieldLayout;

  private Pigeon2 pigeon = new Pigeon2(0, "Canivore");
  private Pigeon2Configuration pigeonConfig = new Pigeon2Configuration();
  Transform3d robotToCam = new Transform3d(
      new Translation3d(Constants.inchesToMeters(1.75), Constants.inchesToMeters(11.625),
          Constants.inchesToMeters(33.5)),
      new Rotation3d(0, Math.toRadians(30.6), 0));
  PhotonPoseEstimator photonPoseEstimator;

  double pigeonSetpoint = 0.0;

  boolean frontReefCamTrack = false;
  boolean backReefCamTrack = false;
  boolean frontBargeCamTrack = false;
  boolean backBargeCamTrack = false;

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

    setPigeonPitchOffset(getPigeonPitch());

  }

  public double getFrontReefCamYaw() {
    double yaw = 0.0;
    var result = frontReefCam.getLatestResult();
    // Logger.recordOutput("has target", result.hasTargets());
    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      yaw = target.getYaw();
    }
    return yaw;
  }

  public double getFrontReefCamPitch() {
    double pitch = 0.0;
    var result = frontReefCam.getLatestResult();
    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      pitch = target.getPitch();
    }

    return pitch;
  }

  public void setPipelineIndex(int index) {
    gamePieceCamera.setPipelineIndex(index);
  }

  public double getGamePieceCamYaw() {
    double yaw = 0.0;
    var result = gamePieceCamera.getLatestResult();
    // Logger.recordOutput("has target", result.hasTargets());
    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      yaw = target.getYaw();
    }
    return yaw;
  }

  // public List<TargetCorner> getGamePieceCamCorners() {
  // List<PhotonPipelineResult> results = gamePieceCamera.getAllUnreadResults();

  // if (!results.isEmpty() && results.get(0).hasTargets()) {
  // PhotonTrackedTarget target = results.get(0).getBestTarget();
  // List<TargetCorner> corners = target.getDetectedCorners();

  // if (corners != null && !corners.isEmpty()) {
  // return corners;
  // }
  // }

  // List<TargetCorner> defaultCorners = new ArrayList<>();
  // for (int i = 0; i < 4; i++) {
  // defaultCorners.add(new TargetCorner(-1, -1));
  // }
  // return defaultCorners;
  // }

  private List<TargetCorner> CoralCorners = new ArrayList<>();

  public void updateDetectionCorners() {
    var result = getFrontGamePieceCamResult();

    if (result.hasTargets()) {
      List<PhotonTrackedTarget> tracks = new ArrayList<>(result.getTargets());

      if (!tracks.isEmpty()) {
        PhotonTrackedTarget bestTrack = tracks.get(0);

        for (PhotonTrackedTarget track : tracks) {
          if (track.getPitch() < bestTrack.getPitch()) {
            bestTrack = track;
          }
        }

        List<TargetCorner> corners = result.getBestTarget().minAreaRectCorners;
        if (corners != null && !corners.isEmpty()) {
          CoralCorners = corners;
          return;
        }
      }
    }

    CoralCorners = getDefaultCorners();
  }

  private double frontCoralLabel = 3;
  private double backCoralLabel = 4;

  private double lastValidAngle = Double.NaN;
  private double lastFrontAngle = Double.NaN;
  private double lastBackAngle = Double.NaN;

  private List<PhotonTrackedTarget> frontCoralList = new ArrayList<>();
  private List<PhotonTrackedTarget> coralList = new ArrayList<>();
  private List<PhotonTrackedTarget> backCoralList = new ArrayList<>();

  public double getCoralAngle(double rawAngle) {
    try {
      var result = gamePieceCamera.getLatestResult();
      if (result == null || result.targets == null) {
        return fallbackAngle(rawAngle);
      }

      frontCoralList.removeIf(t -> result.targets.stream()
          .noneMatch(ct -> ct.equals(t) && ct.objDetectId == frontCoralLabel));
      for (PhotonTrackedTarget target : result.targets) {
        if (target.objDetectId == frontCoralLabel && !frontCoralList.contains(target)) {
          frontCoralList.add(target);
        }
      }

      coralList.removeIf(t -> result.targets.stream()
          .noneMatch(ct -> ct.equals(t) && ct.objDetectId == 2));
      for (PhotonTrackedTarget target : result.targets) {
        if (target.objDetectId == 2 && !coralList.contains(target)) {
          coralList.add(target);
        }
      }

      backCoralList.removeIf(t -> result.targets.stream()
          .noneMatch(ct -> ct.equals(t) && ct.objDetectId == backCoralLabel));
      for (PhotonTrackedTarget target : result.targets) {
        if (target.objDetectId == backCoralLabel && !backCoralList.contains(target)) {
          backCoralList.add(target);
        }
      }

      double adjustedAngle = normalizeAngle(rawAngle);

      if (!frontCoralList.isEmpty() && !coralList.isEmpty()) {
        PhotonTrackedTarget front = frontCoralList.get(0);
        PhotonTrackedTarget normal = coralList.get(0);
        if (hasValidCorner(front) && hasValidCorner(normal)) {
          if (front.getDetectedCorners().get(0).x > normal.getDetectedCorners().get(0).x) {
            adjustedAngle = normalizeAngle(180.0 - rawAngle);
          } else {
            adjustedAngle = normalizeAngle(rawAngle);
          }
        }
        if (isValidAngle(adjustedAngle)) {
          lastFrontAngle = adjustedAngle;
        } else {
          adjustedAngle = lastFrontAngle;
        }
      } else if (!backCoralList.isEmpty() && !coralList.isEmpty()) {
        PhotonTrackedTarget back = backCoralList.get(0);
        PhotonTrackedTarget normal = coralList.get(0);
        if (hasValidCorner(back) && hasValidCorner(normal)) {
          if (back.getDetectedCorners().get(0).x < normal.getDetectedCorners().get(0).x) {
            adjustedAngle = normalizeAngle(180.0 - rawAngle);
          } else {
            adjustedAngle = normalizeAngle(rawAngle);
          }
        }
        if (isValidAngle(adjustedAngle)) {
          lastBackAngle = adjustedAngle;
        } else {
          adjustedAngle = lastBackAngle;
        }
      } else if (!coralList.isEmpty()) {
        adjustedAngle = normalizeAngle(rawAngle);
      } else {
        adjustedAngle = normalizeAngle(rawAngle);
      }

      if (isValidAngle(adjustedAngle)) {
        lastValidAngle = adjustedAngle;
      } else {
        adjustedAngle = lastValidAngle;
      }

      return adjustedAngle;
    } catch (Exception e) {
      e.printStackTrace();
      return lastValidAngle;
    }
  }

  private double normalizeAngle(double angle) {
    angle %= 360.0;
    if (angle < 0)
      angle += 360.0;
    return angle;
  }

  private boolean isValidAngle(double angle) {
    return angle != 0.0 && !Double.isInfinite(angle) && !Double.isNaN(angle);
  }

  private double fallbackAngle(double rawAngle) {
    double norm = normalizeAngle(rawAngle);
    return isValidAngle(norm) ? norm : lastValidAngle;
  }

  private boolean hasValidCorner(PhotonTrackedTarget target) {
    return target != null && target.getDetectedCorners() != null && !target.getDetectedCorners().isEmpty();
  }

  private List<TargetCorner> getDefaultCorners() {
    List<TargetCorner> defaultCorners = new ArrayList<>();
    for (int i = 0; i < 4; i++) {
      defaultCorners.add(new TargetCorner(-1, -1));
    }
    return defaultCorners;
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

  // private Pose2d getRobotPoseViaTrig(PhotonTrackedTarget trackedTarget,
  // double[] cameraPositionOnRobot,
  // double robotAngle) {
  // double pitch = trackedTarget.getPitch();
  // double yaw = trackedTarget.getYaw();
  // int id = trackedTarget.getFiducialId();
  // double cameraXOffset = cameraPositionOnRobot[0];
  // double cameraYOffset = cameraPositionOnRobot[1];
  // double cameraZOffset = cameraPositionOnRobot[2];
  // double cameraRYOffset = cameraPositionOnRobot[4];
  // double cameraRZOffset = cameraPositionOnRobot[5];

  // double[] tagPose = Constants.Vision.TAG_POSES[id - 1];
  // double tagHeight = tagPose[2];
  // double tagX = tagPose[0];
  // double tagY = tagPose[1];
  // double tagYaw = tagPose[3];

  // double distToTag = (tagHeight - cameraZOffset) /
  // Math.tan(Math.toRadians(pitch + cameraRYOffset));
  // Logger.recordOutput("Distance to Tag", distToTag);
  // Logger.recordOutput("yaw to Tag", yaw);
  // double txProjOntoGroundPlane = Math.atan((Math.tan(yaw)) / Math.cos(pitch));
  // double xFromTag = distToTag * Math.cos(Math.toRadians(txProjOntoGroundPlane +
  // robotAngle + cameraRZOffset));
  // double yFromTag = distToTag * Math.sin(Math.toRadians(txProjOntoGroundPlane +
  // robotAngle + cameraRZOffset));
  // Logger.recordOutput("x to Tag", xFromTag);
  // Logger.recordOutput("y to Tag", yFromTag);

  // double fieldPoseX = -xFromTag + tagX - cameraXOffset;
  // double fieldPoseY = -yFromTag + tagY - cameraYOffset;
  // Pose2d pose = new Pose2d(fieldPoseX, fieldPoseY, new
  // Rotation2d(Math.toRadians(getPigeonAngle())));
  // return pose;
  // }

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
  // public static Pose3d calculateRobotPosition(
  // Pose3d cameraOffset, double horizontalAngle, double verticalAngle, Pose3d
  // tagPosition, double robotYaw) {

  // // Extract the camera's offset from the robot in its local frame
  // Translation3d cameraTranslation = cameraOffset.getTranslation();

  // // Rotate the camera's offset into the field frame using the robot's yaw
  // double cosYaw = Math.cos(robotYaw);
  // double sinYaw = Math.sin(robotYaw);
  // Translation3d cameraInFieldTranslation = new Translation3d(
  // cosYaw * cameraTranslation.getX() - sinYaw * cameraTranslation.getY(),
  // sinYaw * cameraTranslation.getX() + cosYaw * cameraTranslation.getY(),
  // cameraTranslation.getZ());

  // // Calculate the relative position of the tag to the camera
  // double dx = tagPosition.getTranslation().getX() -
  // cameraInFieldTranslation.getX();
  // double dy = tagPosition.getTranslation().getY() -
  // cameraInFieldTranslation.getY();
  // double dz = tagPosition.getTranslation().getZ() -
  // cameraInFieldTranslation.getZ();

  // // Calculate the distance from the camera to the tag
  // double distanceToTag = Math.sqrt(dx * dx + dy * dy + dz * dz);

  // // Translation from the camera to the tag in the camera's local space
  // Translation3d tagInCameraSpace = new Translation3d(
  // distanceToTag * Math.cos(horizontalAngle) * Math.cos(verticalAngle), // X
  // distanceToTag * Math.sin(horizontalAngle) * Math.cos(verticalAngle), // Y
  // distanceToTag * Math.sin(verticalAngle) // Z
  // );

  // // Transform the tag position from the camera space to the field space
  // Translation3d tagInFieldSpace =
  // cameraInFieldTranslation.plus(tagInCameraSpace);

  // // Calculate the robot's position in the field frame
  // Translation3d robotTranslation =
  // tagPosition.getTranslation().minus(tagInFieldSpace);

  // // Return the robot's position with its yaw in the field coordinate system
  // return new Pose3d(robotTranslation, new Rotation3d(0, 0, robotYaw));
  // }

  // public Pose2d getFrontReefCamTrigPose() {
  // var result = frontReefCam.getLatestResult();
  // if (result.hasTargets() && result.getBestTarget().getPoseAmbiguity() < 0.3) {
  // PhotonTrackedTarget target = result.getBestTarget();
  // // Pose3d robotPose = calculateRobotPosition(cameraOffset, target.getYaw(),
  // // target.getPitch(),
  // // new Pose3d(
  // // new Translation3d(Constants.Vision.TAG_POSES[10][0],
  // // Constants.Vision.TAG_POSES[10][1],
  // // Constants.Vision.TAG_POSES[10][2]),
  // // new Rotation3d(0.0, Constants.Vision.TAG_POSES[10][4],
  // // Constants.Vision.TAG_POSES[10][3])),
  // // Math.toRadians(getPigeonAngle()));
  // Pose2d robotPose = getRobotPoseViaTrig(target,
  // Constants.Vision.FRONT_CAMERA_POSE, getPigeonAngle());
  // Logger.recordOutput("Trig Localiazation", robotPose);
  // return robotPose;
  // } else {
  // Pose2d defaultPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
  // return defaultPose;
  // }
  // }
  public void switchPipeline(int pipeline) {
    gamePieceCamera.setPipelineIndex(pipeline);
  }

  public PhotonPipelineResult getFrontReefCamResult() {
    var result = frontReefCam.getAllUnreadResults();
    if (!result.isEmpty()) {
      frontReefCamTrack = true;
      return result.get(0);
    } else {
      frontReefCamTrack = false;
      return new PhotonPipelineResult();
    }
  }

  public PhotonPipelineResult getBackReefCamResult() {
    var result = backReefCam.getAllUnreadResults();
    if (!result.isEmpty()) {
      backReefCamTrack = true;
      return result.get(0);
    } else {
      backReefCamTrack = false;
      return new PhotonPipelineResult();
    }
  }

  public PhotonPipelineResult getFrontBargeCamResult() {
    var result = frontBargeCam.getAllUnreadResults();
    if (!result.isEmpty()) {
      frontBargeCamTrack = true;
      return result.get(0);
    } else {
      frontBargeCamTrack = false;
      return new PhotonPipelineResult();
    }
  }

  public PhotonPipelineResult getBackBargeCamResult() {
    var result = backBargeCam.getAllUnreadResults();
    if (!result.isEmpty()) {
      backBargeCamTrack = true;
      return result.get(0);
    } else {
      backBargeCamTrack = false;
      return new PhotonPipelineResult();
    }
  }

  public PhotonPipelineResult getFrontGamePieceCamResult() {
    var result = gamePieceCamera.getAllUnreadResults();
    if (!result.isEmpty()) {
      return result.get(0);
    } else {
      return new PhotonPipelineResult();
    }
  }

  // public Pose3d getFrontReefCamPnPPose() {
  // // Logger.recordOutput("April Tag", aprilTagFieldLayout.getTagPose(1).get());
  // var result = frontReefCam.getLatestResult();
  // Optional<EstimatedRobotPose> multiTagResult =
  // photonPoseEstimator.update(result);
  // if (multiTagResult.isPresent()) {
  // Pose3d robotPose = multiTagResult.get().estimatedPose;
  // Logger.recordOutput("multitag result", robotPose);
  // return robotPose;
  // } else {
  // Pose3d robotPose = new Pose3d();
  // return robotPose;
  // }
  // // var result = frontReefCam.getLatestResult();
  // // Logger.recordOutput("Is presetn", result.getMultiTagResult().isPresent());
  // // if (result.getMultiTagResult().isPresent()) {
  // // Transform3d fieldToCamera =
  // // result.getMultiTagResult().get().estimatedPose.best;
  // // Logger.recordOutput("Camera pose: ", fieldToCamera);
  // // return fieldToCamera;
  // // } else {
  // // System.out.println("in else");
  // // return new Transform3d();
  // // }
  // }

  public double getFrontReefCamLatency() {
    return frontReefCam.getLatestResult().getTimestampSeconds();
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

  public double getPigeonPitch() {
    return pigeon.getPitch().getValueAsDouble();
  }

  public double getPigeonPitchAdjusted() {
    return getPigeonPitch() - pigeonPitchOffset;
  }

  double pigeonPitchOffset = 0.0;

  public void setPigeonPitchOffset(double newOffset) {
    pigeonPitchOffset = newOffset;
  }

  public static double calculateAngle(double x) {
    return 102 + (-25.6 * x) + (-7.16 * Math.pow(x, 2) - 10 * x / 90 / 2);
  }

  public void periodic() {
    updateDetectionCorners();
    double coralAngle = calculateAngle(
        (CoralCorners.get(1).x - CoralCorners.get(0).x) / (CoralCorners.get(3).y - CoralCorners.get(0).y));
    Logger.recordOutput("Coral Angle", getCoralAngle(coralAngle));
    Logger.recordOutput("Current Pipeline", gamePieceCamera.getPipelineIndex());

    // Logger.recordOutput("Coral Area", getCoralArea());
    // Logger.recordOutput("Bottom Left Corner", CoralCorners.get(0));
    // Logger.recordOutput("Bottom Right Corner", CoralCorners.get(1));
    // Logger.recordOutput("Top Right Corner", CoralCorners.get(2));
    // Logger.recordOutput("Top Left Corner", CoralCorners.get(3));

    // Logger.recordOutput("1length", CoralCorners.get(1).x -
    // CoralCorners.get(0).x);
    // Logger.recordOutput("1width", CoralCorners.get(3).y - CoralCorners.get(0).y);
    Logger.recordOutput("LW Ratio",
        (CoralCorners.get(1).x - CoralCorners.get(0).x) / (CoralCorners.get(3).y -
            CoralCorners.get(0).y));
    // Logger.recordOutput("Total Detections",
    // gamePieceCamera.getAllUnreadResults().size());
    Logger.recordOutput("Pigeon Pitch", getPigeonPitchAdjusted());

    // Use to take snapshots of camera stream (Output means processed stream, input
    // means raw stream)
    // if (Timer.getFPGATimestamp() - cameraScreenshotTime > 1.0 &&
    // (DriverStation.isEnabled())) {
    // gamePieceCamera.takeOutputSnapshot();
    // cameraScreenshotTime = Timer.getFPGATimestamp();
    // }

    // Logger.recordOutput("Pidgeon Yaw?", pigeon.getYaw().getValueAsDouble());
    // Logger.recordOutput("Pidgeon Pitch?", pigeon.getPitch().getValueAsDouble());
    // Logger.recordOutput("Pidgeon Roll?", pigeon.getRoll().getValueAsDouble());
    // TODO: uncomment if you want to see if the cameras have a track
    // Logger.recordOutput("Front Cam Track", frontReefCamTrack);
    // Logger.recordOutput("Back Cam Track", backReefCamTrack);
    // Logger.recordOutput("Right Cam Track", frontBargeCamTrack);
    // Logger.recordOutput("Left Cam Track", backBargeCamTrack);

    // getFrontReefCamPnPPose(); //TODO: uncomment when using camera
    // var result = frontReefCam.getLatestResult();
    // if (result.hasTargets()) {
    // PhotonTrackedTarget target = result.getBestTarget();
    // Pose2d robotPose = getRobotPoseViaTrig(target,
    // Constants.Vision.FRONT_CAMERA_POSE, getPigeonAngle());
    // Logger.recordOutput("Trig Localiazation", robotPose);
    // }
  }
}