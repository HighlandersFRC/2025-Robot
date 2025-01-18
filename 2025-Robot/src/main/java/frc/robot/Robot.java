package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.MotorTest;
import frc.robot.subsystems.Superstructure.SuperState;

public class Robot extends LoggedRobot {
  private RobotContainer m_robotContainer = new RobotContainer();
  private Command m_autonomousCommand;

  private MotorTest motortest = new MotorTest();
  String m_fieldSide = "blue";

  @Override
  public void robotInit() {

    System.out.println("Robot Init");
    Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    Logger.start();

    this.m_fieldSide = "blue";
    SmartDashboard.putNumber("Shooter Angle Degrees (tuning)", 0);
    SmartDashboard.putNumber("Shooter RPM (input)", 0);

    m_robotContainer.peripherals.init();
    m_robotContainer.drive.init(m_fieldSide);
    m_robotContainer.elevator.init();
    motortest.init();
    Constants.init();

    PortForwarder.add(5800, "orangepi1.local", 5800);
    PortForwarder.add(5801, "orangepi1.local", 5801);

    PortForwarder.add(5800, "10.44.99.34", 5800);
    PortForwarder.add(5801, "10.44.99.34", 5801);

    m_robotContainer.lights.clearAnimations();
    m_robotContainer.lights.setCommandRunning(true);
    m_robotContainer.lights.setRGBFade();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    try {
      Logger.recordOutput("Localization Odometry", m_robotContainer.drive.getLocalizationOdometry());
    } catch (Exception e) {
      System.out.println("Problem with logging");
    }

    try {
      Logger.recordOutput("Wheel Odometry", m_robotContainer.drive.getOdometry());
    } catch (Exception e) {
      System.out.println("Problem with logging");
    }

    try {
      Logger.recordOutput("MT2 Odometry", m_robotContainer.drive.getMT2Odometry());
    } catch (Exception e) {
      System.out.println("Problem with logging");
    }
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
    m_robotContainer.lights.setCommandRunning(true);
    m_robotContainer.lights.setRainbow();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.superstructure.setWantedState(SuperState.IDLE);
    if (OI.isBlueSide()) {
      System.out.println("ON BLUE SIDE");
      m_fieldSide = "blue";
    } else {
      System.out.println("ON RED SIDE");
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
    m_robotContainer.elevator.teleopInit();
    m_robotContainer.lights.setCommandRunning(false);
    m_robotContainer.lights.clearAnimations();
    m_robotContainer.superstructure.setWantedState(SuperState.CYCLING);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    if (OI.isBlueSide()) {
      m_fieldSide = "blue";
    } else {
      m_fieldSide = "red";
    }
    if (this.m_fieldSide == "red") {
      this.m_robotContainer.drive.setPigeonAfterAuto();
    }
    System.out.println("field side" + m_fieldSide);

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
