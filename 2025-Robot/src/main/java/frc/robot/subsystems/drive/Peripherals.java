package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.tools.math.Vector;

public class Peripherals {
  private PhotonCamera frontReefCam = new PhotonCamera("Front_Reef");
  private PhotonCamera frontSwerveCam = new PhotonCamera("Front_Swerve");
  private PhotonCamera backReefCam = new PhotonCamera("Back_Reef");
  private PhotonCamera backLeftReefCam = new PhotonCamera("Back_Left_Reef");
  private PhotonCamera backRightReefCam = new PhotonCamera("Back_Right_Reef");
  private PhotonCamera frontBargeCam = new PhotonCamera("Front_Barge");
  private PhotonCamera backBargeCam = new PhotonCamera("Back_Barge");
  private PhotonCamera gamePieceCamera = new PhotonCamera("Front_Game_Piece_Cam");

  AprilTagFieldLayout aprilTagFieldLayout;

  private Pigeon2 pigeon = new Pigeon2(0, "Canivore");
  private Pigeon2 pigeonExtra = new Pigeon2(1, "Canivore");

  private Pigeon2Configuration pigeonConfig = new Pigeon2Configuration();
  private Pigeon2Configuration pigeonExtraConfig = new Pigeon2Configuration();
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
      java.util.logging.Logger.getGlobal().warning("error with april tag: " + e.getMessage());
    }
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
    // Set the mount pose configuration for the IMU
    pigeonConfig.MountPose.MountPosePitch = 0.3561480641365051;
    pigeonConfig.MountPose.MountPoseRoll = -0.10366992652416229;
    pigeonConfig.MountPose.MountPoseYaw = -0.24523599445819855;

    pigeonExtraConfig.MountPose.MountPosePitch = 2.9378318786621094;
    pigeonExtraConfig.MountPose.MountPoseRoll = -1.7237101793289185;
    pigeonExtraConfig.MountPose.MountPoseYaw = -1.0769075155258179;

    // Apply the IMU configuration
    pigeon.getConfigurator().apply(pigeonConfig);
    pigeonExtra.getConfigurator().apply(pigeonExtraConfig);

    // Zero the IMU angle
    zeroPigeon();

    setPigeonPitchOffset(getPigeonPitch());

  }

  public void setBackCamPipline(int index) {
    backReefCam.setPipelineIndex(index);
  }

  public void setGamePieceCamPipline(int index) {
    gamePieceCamera.setPipelineIndex(index);
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

  public PhotonPipelineResult getBackLeftReefCamResult() {
    var result = backLeftReefCam.getAllUnreadResults();
    if (!result.isEmpty()) {
      return result.get(0);
    } else {
      return new PhotonPipelineResult();
    }
  }

  public PhotonPipelineResult getBackRightReefCamResult() {
    var result = backRightReefCam.getAllUnreadResults();
    if (!result.isEmpty()) {
      return result.get(0);
    } else {
      return new PhotonPipelineResult();
    }
  }

  public PhotonPipelineResult getFrontSwerveCamResult() {
    var result = frontSwerveCam.getAllUnreadResults();
    if (!result.isEmpty()) {
      return result.get(0);
    } else {
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
    pigeonExtra.setYaw(degrees);
  }

  /**
   * Retrieves the yaw of the robot
   * 
   * @return Yaw in degrees
   */
  public double getPigeonAngle() {
    return pigeon.getYaw().getValueAsDouble();
  }

  public double getPigeonExtraAngle() {
    return pigeonExtra.getYaw().getValueAsDouble();
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

  double cameraScreenshotTime = 0.0;

  public void periodic() {
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
  }
}