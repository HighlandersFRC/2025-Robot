package frc.robot.commands;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;

import org.json.JSONArray;
import org.json.JSONObject;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.tools.math.Spline;
import frc.robot.tools.math.Vector;
import frc.robot.tools.utils.PolarPathing;
import frc.robot.tools.wrappers.AutoFollower;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Drive;

public class VariableSpeedFollower extends AutoFollower {
  private Drive drive;

  private JSONArray path;
  private Spline spline;

  private double initTime;
  private double currentTime;

  private double odometryFusedX = 0;
  private double odometryFusedY = 0;
  private double odometryFusedTheta = 0;

  private Number[] desiredVelocityArray = new Number[4];
  private double desiredThetaChange = 0;

  private boolean record;

  private ArrayList<double[]> recordedOdometry = new ArrayList<double[]>();
  public double pathStartTime;

  private int prevTargetIndex = 0;
  private int closestIndex = 0;
  private int targetIndex = 0;
  private int timesStagnated = 0;
  private final int STAGNATE_THRESHOLD = 3;
  private boolean reset = true;
  private int endIndex = 0;

  public int getPathPointIndex() {
    return closestIndex;
  }

  public VariableSpeedFollower(Drive drive, JSONObject path,
      boolean record) {
    this.drive = drive;
    this.path = path.getJSONArray("sampled_points");
    this.spline = PolarPathing.pathJsonToSpline(path);
    this.record = record;
    pathStartTime = this.path.getJSONObject(0).getDouble("time");
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    pathStartTime = path.getJSONObject(0).getDouble("time");
    initTime = Timer.getFPGATimestamp();
    if (reset) {
      this.endIndex = path.length() - 1;
      prevTargetIndex = 0;
    } else {
      reset = true;
    }
    if (endIndex > path.length() - 1) {
      endIndex = path.length() - 1;
    }
    targetIndex = prevTargetIndex;
    timesStagnated = 0;
  }

  @Override
  public void execute() {
    drive.updateOdometryFusedArray();
    odometryFusedX = drive.getMT2OdometryX();
    odometryFusedY = drive.getMT2OdometryY();
    odometryFusedTheta = drive.getMT2OdometryAngle();
    currentTime = Timer.getFPGATimestamp() - initTime + pathStartTime;
    // call PIDController function
    prevTargetIndex = targetIndex;
    desiredVelocityArray = drive.purePursuitController(odometryFusedX, odometryFusedY, odometryFusedTheta,
        prevTargetIndex, path, spline, false, false);

    closestIndex = desiredVelocityArray[3].intValue();
    targetIndex = desiredVelocityArray[4].intValue();
    if (targetIndex == prevTargetIndex && targetIndex != path.length() - 1) {
      timesStagnated++;
      if (timesStagnated > STAGNATE_THRESHOLD) {
        targetIndex++;
        timesStagnated = 0;
      }
    } else {
      timesStagnated = 0;
    }

    Vector velocityVector = new Vector();

    if (prevTargetIndex == path.length() - 1) {
      velocityVector.setI(desiredVelocityArray[0].doubleValue() * 2);
      velocityVector.setJ(desiredVelocityArray[1].doubleValue() * 2);
      desiredThetaChange = desiredVelocityArray[2].doubleValue() * 2;
    } else {
      velocityVector.setI(desiredVelocityArray[0].doubleValue());
      velocityVector.setJ(desiredVelocityArray[1].doubleValue());
      desiredThetaChange = desiredVelocityArray[2].doubleValue();
    }

    // create velocity vector and set desired theta change

    drive.autoDrive(velocityVector, desiredThetaChange);
    Logger.recordOutput("pursuing?", true);
    Logger.recordOutput("Path Point Index", getPathPointIndex());
    // Logger.recordOutput("Path Time", path
    // .getJSONObject(getPathPointIndex()).getDouble("time"));
  }

  @Override
  public void end(boolean interrupted) {
    Vector velocityVector = new Vector();
    velocityVector.setI(0);
    velocityVector.setJ(0);
    double desiredThetaChange = 0.0;
    drive.autoDrive(velocityVector, desiredThetaChange);

    odometryFusedX = drive.getFusedOdometryX();
    odometryFusedY = drive.getFusedOdometryY();
    odometryFusedTheta = drive.getFusedOdometryTheta();
    currentTime = Timer.getFPGATimestamp() - initTime;
    Logger.recordOutput("pursuing?", false);
    if (this.record) {
      recordedOdometry.add(new double[] { currentTime, odometryFusedX, odometryFusedY, odometryFusedTheta });
      try {
        DateTimeFormatter dtf = DateTimeFormatter.ofPattern("yyyy-MM-dd-hh-mm-ss");
        LocalDateTime now = LocalDateTime.now();
        String filename = "/home/lvuser/deploy/recordings/" + dtf.format(now) + ".csv";
        File file = new File(filename);
        if (!file.exists()) {
          file.createNewFile();
        }
        FileWriter fw = new FileWriter(file);
        BufferedWriter bw = new BufferedWriter(fw);
        for (int i = 0; i < recordedOdometry.size(); i++) {
          String line = "";
          for (double val : recordedOdometry.get(i)) {
            line += val + ",";
          }
          line = line.substring(0, line.length() - 1);
          line += "\n";
          bw.write(line);
        }

        bw.close();
      } catch (Exception e) {
        java.util.logging.Logger.getGlobal().warning(e.getMessage());
        java.util.logging.Logger.getGlobal().warning("CSV file error");
      }
    }
  }

  public void from(int pointIndex, JSONObject pathJSON, int to) {
    this.prevTargetIndex = pointIndex;
    this.closestIndex = pointIndex;
    path = pathJSON.getJSONArray("sampled_points");
    spline = PolarPathing.pathJsonToSpline(pathJSON);
    endIndex = to;
    reset = false;
  }

  @Override
  public boolean isFinished() {
    boolean ready = readyToEnd(path.getJSONObject(targetIndex));
    Logger.recordOutput("Ready to end", ready);
    if (targetIndex >= path.length() - 1 && ready) {
      return true;
    } else {
      return false;
    }
  }

  private boolean readyToEnd(JSONObject point) {
    double odometryFusedX = drive.getMT2OdometryX();
    double odometryFusedY = drive.getMT2OdometryY();
    double odometryFusedTheta = drive.getMT2OdometryAngle();
    if (drive.getFieldSide() == "blue") {
      odometryFusedX = Constants.Physical.FIELD_LENGTH - odometryFusedX;
      odometryFusedY = Constants.Physical.FIELD_WIDTH - odometryFusedY;
      odometryFusedTheta = Math.PI + odometryFusedTheta;
    }

    if (OI.isProcessorSide()) {
      odometryFusedY = Constants.Physical.FIELD_WIDTH - odometryFusedY;
      odometryFusedTheta = -odometryFusedTheta;
    }
    while (Math.abs(odometryFusedTheta - point.getDouble("angle")) > Math.PI) {
      if (odometryFusedTheta - point.getDouble("angle") > Math.PI) {
        odometryFusedTheta -= 2 * Math.PI;
      } else if (odometryFusedTheta - point.getDouble("angle") < -Math.PI) {
        odometryFusedTheta += 2 * Math.PI;
      }
    }
    return drive.insideRadius(
        (point.getDouble("x") - odometryFusedX) / Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS,
        (point.getDouble("y") - odometryFusedY) / Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS,
        (point.getDouble("angle") - odometryFusedTheta) / Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_ANGULAR_RADIUS,
        Constants.Autonomous.AUTONOMOUS_END_ACCURACY);
  }
}
