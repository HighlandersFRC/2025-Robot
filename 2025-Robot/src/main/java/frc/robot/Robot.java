package frc.robot;

import java.util.logging.Level;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.tools.logging.AdvantageKitMultiLevelLogHandler;

public class Robot extends LoggedRobot {
  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;

  String m_fieldSide = "blue";
  boolean bPressed = false;
  boolean yPressed = false;
  boolean xPressed = false;
  boolean autoChooserCenterSwitch = false;

  @Override
  public void robotInit() {
    /*
     * The Logging Framework built into Java has 5 levels of logging:
     * 
     * Severe: Used for very serious errors that will cause the program to crash
     * (e.g. Auto not loaded).
     * 
     * Warning: Used for potentially harmful situations (e.g. Cameras unable to load
     * Field Layout).
     * 
     * Info: Used for informational messages that highlight the progress of the
     * match (e.g. Auto chose True Path, Robot Init).
     * 
     * Fine: Used for debugging messages that are useful for developers, but don't
     * print every scheduler run (e.g. robot state set to climb).
     * 
     * Finer: Used for very detailed debugging messages that print every scheduler
     * run (e.g. robot position, elevator height)
     * 
     * For Match Logging, use Info Level, and all the other stuff save to Advantage
     * Scope
     * For Development, use Fine or Finer
     * 
     */

    Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    Logger.start();

    // The level for logs going to advantage scope. LEAVE THIS AT "ALL"
    java.util.logging.Logger.getLogger("").setLevel(Level.ALL);

    // The level for logs printed to console. CHANGE THIS ONE TO OFF FOR COMP
    java.util.logging.Logger.getLogger("").getHandlers()[0].setLevel(Level.INFO);

    java.util.logging.Logger.getLogger("").addHandler(new AdvantageKitMultiLevelLogHandler());

    java.util.logging.Logger.getGlobal().info("Robot Init");

    this.m_fieldSide = "blue";
    SmartDashboard.putNumber("Shooter Angle Degrees (tuning)", 0);
    SmartDashboard.putNumber("Shooter RPM (input)", 0);
    m_robotContainer = new RobotContainer();

    m_robotContainer.peripherals.init();
    m_robotContainer.drive.init(m_fieldSide);
    m_robotContainer.elevator.init();
    m_robotContainer.pivot.init();
    m_robotContainer.twist.init();
    m_robotContainer.manipulator.init();
    m_robotContainer.climber.init();
    m_robotContainer.intake.init();
    m_robotContainer.lights.init(m_fieldSide);

    PortForwarder.add(5800, "orangepi1.local", 5800);
    PortForwarder.add(5801, "orangepi1.local", 5801);

    PortForwarder.add(5800, "10.44.99.34", 5800);
    PortForwarder.add(5801, "10.44.99.34", 5801);

    m_robotContainer.lights.clearAnimations();
    SmartDashboard.putNumber("L2/3 Front X", Constants.metersToInches(Constants.Physical.INTAKE_X_OFFSET_FRONT));
    SmartDashboard.putNumber("L2/3 Front Y", Constants.metersToInches(Constants.Physical.INTAKE_Y_OFFSET_FRONT));
    SmartDashboard.putNumber("L2/3 Back X", Constants.metersToInches(Constants.Physical.INTAKE_X_OFFSET_BACK));
    SmartDashboard.putNumber("L2/3 Back Y", Constants.metersToInches(Constants.Physical.INTAKE_Y_OFFSET_BACK));
    SmartDashboard.putNumber("L4 Front X", Constants.metersToInches(Constants.Physical.L4_INTAKE_X_OFFSET_FRONT));
    SmartDashboard.putNumber("L4 Front Y", Constants.metersToInches(Constants.Physical.L4_INTAKE_Y_OFFSET_FRONT));
    SmartDashboard.putNumber("L4 Back X", Constants.metersToInches(Constants.Physical.L4_INTAKE_X_OFFSET_BACK));
    SmartDashboard.putNumber("L4 Back Y", Constants.metersToInches(Constants.Physical.L4_INTAKE_Y_OFFSET_BACK));
    SmartDashboard.putNumber("Algae Front X", Constants.metersToInches(Constants.Physical.INTAKE_X_OFFSET_FRONT_ALGAE));
    SmartDashboard.putNumber("Algae Front Y", Constants.metersToInches(Constants.Physical.INTAKE_Y_OFFSET_FRONT_ALGAE));
    SmartDashboard.putNumber("Algae Back X", Constants.metersToInches(Constants.Physical.INTAKE_X_OFFSET_BACK_ALGAE));
    SmartDashboard.putNumber("Algae Back Y", Constants.metersToInches(Constants.Physical.INTAKE_Y_OFFSET_BACK_ALGAE));

    // m_robotContainer.lights.setFlashYellow();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    try {
      Logger.recordOutput("Localization Odometry", m_robotContainer.drive.getLocalizationOdometry());
    } catch (Exception e) {
      java.util.logging.Logger.getGlobal().severe("Problem with logging");
    }

    try {
      Logger.recordOutput("Wheel Odometry", m_robotContainer.drive.getOdometry());
    } catch (Exception e) {
      java.util.logging.Logger.getGlobal().severe("Problem with logging");
    }

    try {
      Logger.recordOutput("MT2 Odometry", m_robotContainer.drive.getMT2Odometry());
    } catch (Exception e) {
      java.util.logging.Logger.getGlobal().severe("Problem with logging");
    }

    // if (OI.isManualMode()) {
    // m_robotContainer.manualMode = true;
    // } else {
    // m_robotContainer.manualMode = false;
    // }

    if (OI.driverB.getAsBoolean()) {
      if (bPressed) {
        m_robotContainer.algaeMode = !m_robotContainer.algaeMode;
        bPressed = false;
      }
    } else {
      bPressed = true;
    }

    if (OI.driverX.getAsBoolean()) {
      if (xPressed) {
        m_robotContainer.manualMode = !m_robotContainer.manualMode;
        xPressed = false;
      }
    } else {
      xPressed = true;
    }

    if (OI.driverY.getAsBoolean()) {
      if (yPressed) {
        m_robotContainer.yPressed = !m_robotContainer.yPressed;
        yPressed = false;
      }
    } else {
      yPressed = true;
    }

    if (OI.isRecalculateMode()) {
      if (autoChooserCenterSwitch) {
        Constants.Reef.calculateReefPoints();
        autoChooserCenterSwitch = false;
      }
    } else {
      autoChooserCenterSwitch = true;
    }

    Constants.Physical.INTAKE_X_OFFSET_FRONT = Constants.inchesToMeters(SmartDashboard.getNumber("L2/3 Front X",
        Constants.metersToInches(Constants.Physical.INTAKE_X_OFFSET_FRONT)));

    Constants.Physical.INTAKE_X_OFFSET_FRONT = Constants.inchesToMeters(SmartDashboard.getNumber("L2/3 Front X",
        Constants.metersToInches(Constants.Physical.INTAKE_X_OFFSET_FRONT)));
    Constants.Physical.INTAKE_Y_OFFSET_FRONT = Constants.inchesToMeters(SmartDashboard.getNumber("L2/3 Front Y",
        Constants.metersToInches(Constants.Physical.INTAKE_Y_OFFSET_FRONT)));
    Constants.Physical.INTAKE_X_OFFSET_BACK = Constants.inchesToMeters(SmartDashboard.getNumber("L2/3 Back X",
        Constants.metersToInches(Constants.Physical.INTAKE_X_OFFSET_BACK)));
    Constants.Physical.INTAKE_Y_OFFSET_BACK = Constants.inchesToMeters(SmartDashboard.getNumber("L2/3 Back Y",
        Constants.metersToInches(Constants.Physical.INTAKE_Y_OFFSET_BACK)));
    Constants.Physical.L4_INTAKE_X_OFFSET_FRONT = Constants.inchesToMeters(SmartDashboard.getNumber("L4 Front X",
        Constants.metersToInches(Constants.Physical.L4_INTAKE_X_OFFSET_FRONT)));
    Constants.Physical.L4_INTAKE_Y_OFFSET_FRONT = Constants.inchesToMeters(SmartDashboard.getNumber("L4 Front Y",
        Constants.metersToInches(Constants.Physical.L4_INTAKE_Y_OFFSET_FRONT)));
    Constants.Physical.L4_INTAKE_X_OFFSET_BACK = Constants.inchesToMeters(SmartDashboard.getNumber("L4 Back X",
        Constants.metersToInches(Constants.Physical.L4_INTAKE_X_OFFSET_BACK)));
    Constants.Physical.L4_INTAKE_Y_OFFSET_BACK = Constants.inchesToMeters(SmartDashboard.getNumber("L4 Back Y",
        Constants.metersToInches(Constants.Physical.L4_INTAKE_Y_OFFSET_BACK)));
    Constants.Physical.INTAKE_X_OFFSET_FRONT_ALGAE = Constants.inchesToMeters(SmartDashboard.getNumber("Algae Front X",
        Constants.metersToInches(Constants.Physical.INTAKE_X_OFFSET_FRONT_ALGAE)));
    Constants.Physical.INTAKE_Y_OFFSET_FRONT_ALGAE = Constants.inchesToMeters(SmartDashboard.getNumber("Algae Front Y",
        Constants.metersToInches(Constants.Physical.INTAKE_Y_OFFSET_FRONT_ALGAE)));
    Constants.Physical.INTAKE_X_OFFSET_BACK_ALGAE = Constants.inchesToMeters(SmartDashboard.getNumber("Algae Back X",
        Constants.metersToInches(Constants.Physical.INTAKE_X_OFFSET_BACK_ALGAE)));
    Constants.Physical.INTAKE_Y_OFFSET_BACK_ALGAE = Constants.inchesToMeters(SmartDashboard.getNumber("Algae Back Y",
        Constants.metersToInches(Constants.Physical.INTAKE_Y_OFFSET_BACK_ALGAE)));
    // m_robotContainer.twist.setAlgaeMode(m_robotContainer.algaeMode);
    // m_robotContainer.pivot.setAlgaeMode(m_robotContainer.algaeMode);
    m_robotContainer.superstructure.algaeMode = m_robotContainer.algaeMode;
    m_robotContainer.lights.updateIntakeItem(m_robotContainer.manipulator.getArmItem());
    m_robotContainer.manipulator.updateAlgaeMode(m_robotContainer.algaeMode);
    m_robotContainer.lights.updateAlgaeMode(m_robotContainer.algaeMode);
    m_robotContainer.lights.updateManualMode(m_robotContainer.manualMode);
    if (DriverStation.isAutonomousEnabled()) {
      m_robotContainer.twist.algaeMode = m_robotContainer.algaeMode;
    } else {
      m_robotContainer.twist.algaeMode = false;
    }
    Logger.recordOutput("Algae Mode", m_robotContainer.algaeMode);
    Logger.recordOutput("Manual Mode", m_robotContainer.manualMode);
    Logger.recordOutput("Swerve Module States", m_robotContainer.drive.getModuleStates());
    Logger.recordOutput("Swerve Module Setpoints", m_robotContainer.drive.getModuleSetpoints());
    Logger.recordOutput("IMU", m_robotContainer.peripherals.getPigeonAngle());
    Constants.periodic();
    m_robotContainer.lights.periodic();
    m_robotContainer.peripherals.periodic();
  }

  @Override
  public void disabledInit() {
    OI.driverController.setRumble(RumbleType.kBothRumble, 0);
    OI.operatorController.setRumble(RumbleType.kBothRumble, 0);
    m_robotContainer.lights.clearAnimations();
    java.util.logging.Logger.getGlobal().info("Robot Disabled");
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    double autoInitTime = Timer.getFPGATimestamp();
    m_robotContainer.superstructure.setWantedState(SuperState.IDLE);
    if (OI.isBlueSide()) {
      java.util.logging.Logger.getGlobal().info("ON BLUE SIDE");
      m_fieldSide = "blue";
    } else {
      java.util.logging.Logger.getGlobal().info("ON RED SIDE");
      m_fieldSide = "red";
    }
    this.m_robotContainer.drive.setFieldSide(m_fieldSide);
    this.m_robotContainer.elevator.autoInit();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    java.util.logging.Logger.getGlobal().info("Auto init time" + (Timer.getFPGATimestamp() - autoInitTime));
    m_autonomousCommand.schedule();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    m_robotContainer.elevator.teleopInit();
    m_robotContainer.twist.teleopInit();
    m_robotContainer.lights.clearAnimations();
    m_robotContainer.superstructure.setWantedState(SuperState.ZERO);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    if (OI.isBlueSide()) {
      m_fieldSide = "blue";
    } else {
      m_fieldSide = "red";
    }

    // Leave uncommented to use field relative theta system. Instead we are flipping
    // joystick values on red side.
    // if (this.m_fieldSide == "red") {
    // this.m_robotContainer.drive.setPigeonAfterAuto();
    // }
    java.util.logging.Logger.getGlobal().info("field side" + m_fieldSide);

    this.m_robotContainer.drive.setFieldSide(m_fieldSide);
    this.m_robotContainer.drive.teleopInit();
  }

  @Override
  public void teleopPeriodic() {
    this.m_robotContainer.drive.teleopPeriodic();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
