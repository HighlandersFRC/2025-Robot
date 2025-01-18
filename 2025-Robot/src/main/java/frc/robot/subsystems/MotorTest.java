package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorTest extends SubsystemBase {

  private final int id1 = 20;
  // private final int id2 = 100;
  // private final int id3 = 100;
  // private final int id4 = 100;
  private final String canbus1 = "rio";

  private final TalonFX motor1 = new TalonFX(id1, canbus1);
  // private final TalonFX motor2 = new TalonFX(id2, canbus1);
  // private final TalonFX motor3 = new TalonFX(id3, canbus1);
  // private final TalonFX motor4 = new TalonFX(id4, canbus1);

  public MotorTest() {}

  public void init() {

  }

  public void setMotorPercent(String motorName, double percent) {
    System.out.println("setting percent");
    switch (motorName) {
      case "motor1": 
      System.out.println(percent);
      motor1.set(percent);
      break;
      // case "motor2": 
      // motor2.set(percent);
      // break;
      // case "motor3": 
      // motor3.set(percent);
      // break;
      // case "motor4": 
      // motor4.set(percent);
      // break;
      case "all": 
      System.out.println("all");

      motor1.set(percent);
      // motor2.set(percent);
      // motor3.set(percent);
      // motor4.set(percent);
      break;
      default: 
      System.out.println("Can't find motor with name " + motorName);
      break;
    }
  }



  @Override
  public void periodic() {

  }
}
