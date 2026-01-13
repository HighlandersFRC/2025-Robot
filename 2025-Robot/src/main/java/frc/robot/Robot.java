package frc.robot;

import java.util.logging.Level;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Superstructure.SuperState;

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
    Globals.initTime = Timer.getFPGATimestamp();
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

    // The level for logs going to advantage scope. LEAVE THIS AT "ALL"

    this.m_fieldSide = "blue";
    SmartDashboard.putNumber("Shooter Angle Degrees (tuning)", 0);
    SmartDashboard.putNumber("Shooter RPM (input)", 0);
    m_robotContainer = new RobotContainer();

    m_robotContainer.peripherals.init();
    m_robotContainer.drive.init(m_fieldSide);

    Constants.init();

    PortForwarder.add(5800, "orangepi1.local", 5800);
    PortForwarder.add(5801, "orangepi1.local", 5801);

    PortForwarder.add(5800, "10.44.99.34", 5800);
    PortForwarder.add(5801, "10.44.99.34", 5801);

    // m_robotContainer.lights.setFlashYellow();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.drive.algaeMode = m_robotContainer.algaeMode;
    Globals.loopPeriodSecs = Timer.getFPGATimestamp() - Globals.prevTimeSecs;
    Globals.prevTimeSecs = Timer.getFPGATimestamp();
    Globals.runTime = Timer.getFPGATimestamp() - Globals.initTime;

    m_robotContainer.peripherals.periodic();
  }

  @Override
  public void disabledInit() {
    OI.driverController.setRumble(RumbleType.kBothRumble, 0);
    OI.operatorController.setRumble(RumbleType.kBothRumble, 0);
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    double autoInitTime = Timer.getFPGATimestamp();
    m_robotContainer.superstructure.setWantedState(SuperState.IDLE);
    if (OI.isBlueSide()) {
      m_fieldSide = "blue";
    } else {
      m_fieldSide = "red";
    }
    this.m_robotContainer.drive.setFieldSide(m_fieldSide);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_autonomousCommand.schedule();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
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

    this.m_robotContainer.drive.setFieldSide(m_fieldSide);
    this.m_robotContainer.drive.teleopInit();
  }

  @Override
  public void teleopPeriodic() {
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
        autoChooserCenterSwitch = false;
      }
    } else {
      autoChooserCenterSwitch = true;
    }
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
