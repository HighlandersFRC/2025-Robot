package frc.robot;

import java.io.File;
import java.io.FileReader;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AccurateFollower;
import frc.robot.commands.DoNothing;
import frc.robot.commands.FullSendFollower;
import frc.robot.commands.MoveToPiece;
import frc.robot.commands.MoveToPoint;
import frc.robot.commands.PolarAutoFollower;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunOutake;
import frc.robot.commands.SetElevatorPercent;
import frc.robot.commands.SetElevatorState;
import frc.robot.commands.SetIntake;
import frc.robot.commands.SetPivotPercent;
import frc.robot.commands.SetPivotState;
import frc.robot.commands.SetRobotPose;
import frc.robot.commands.SetRobotState;
import frc.robot.commands.SetRobotStateSimple;
import frc.robot.commands.SetTwistPercent;
import frc.robot.commands.SetTwistState;
import frc.robot.commands.ZeroAngleMidMatch;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.MotorTest;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Twist;
import frc.robot.subsystems.Pivot.PivotState;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.subsystems.Twist.TwistState;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  Peripherals peripherals = new Peripherals();
  Drive drive = new Drive(peripherals);
  Elevator elevator = new Elevator();
  MotorTest motorTest = new MotorTest();
  Intake intake = new Intake();
  Lights lights = new Lights();
  Pivot pivot = new Pivot();
  Twist twist = new Twist();
  Superstructure superstructure = new Superstructure(drive, elevator, intake, pivot, twist);

  HashMap<String, Supplier<Command>> commandMap = new HashMap<String, Supplier<Command>>() {
    {
      // put("Instant", () -> new InstantCommand());
      // put("Outake", () -> new RunOutake(intake, superstructure));
      // put("Elevator Down", () -> new SetRobotStateSimple(superstructure,
      // SuperState.IDLE));
      // put("Elevator L2", () -> new SetRobotStateSimple(superstructure,
      // SuperState.ELEVATOR_L2));
      // put("Elevator Mid", () -> new SetRobotStateSimple(superstructure,
      // SuperState.ELEVATOR_MID));
      // put("Wait", () -> new DoNothing());
      // put("Print", () -> new PrintCommand("10s"));
      // put("Full Send", () -> new FullSendFollower(drive, null, false));
      // put("Accurate", () -> new AccurateFollower(drive, false));
      // put("Place Coral High", () -> new SequentialCommandGroup(
      // new ParallelRaceGroup(
      // new SetRobotState(superstructure, SuperState.ELEVATOR_L2),
      // new WaitCommand(1)),
      // new ParallelRaceGroup(
      // new SetRobotState(superstructure, SuperState.ELEVATOR_MID),
      // new WaitCommand(0.2)),
      // new ParallelRaceGroup(
      // new SetIntake(intake, 0.3),
      // new WaitCommand(0.2))));
      // put("Intake Coral", () -> new RunIntake(intake, superstructure));
      // put("Raise 2in", () -> new SetRobotStateSimple(superstructure,
      // SuperState.ELEVATOR_ALGAE));
    }
  };

  File[] autoFiles = new File[Constants.Autonomous.paths.length];
  Command[] autos = new Command[Constants.Autonomous.paths.length];
  JSONObject[] autoJSONs = new JSONObject[Constants.Autonomous.paths.length];
  JSONArray[] autoPoints = new JSONArray[Constants.Autonomous.paths.length];

  HashMap<String, BooleanSupplier> conditionMap = new HashMap<String, BooleanSupplier>() {
    {
    }
  };

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    // Load the Path Files
    for (int i = 0; i < Constants.Autonomous.paths.length; i++) {
      try {
        autoFiles[i] = new File(Filesystem.getDeployDirectory().getPath() + "/" + Constants.Autonomous.paths[i]);
        FileReader scanner = new FileReader(autoFiles[i]);
        autoJSONs[i] = new JSONObject(new JSONTokener(scanner));
        autoPoints[i] = (JSONArray) autoJSONs[i].getJSONArray("paths").getJSONObject(0).getJSONArray("sampled_points");
        autos[i] = new PolarAutoFollower(autoJSONs[i], drive, lights, peripherals, commandMap, conditionMap);
        System.out.println("Loaded Path: " + Constants.Autonomous.paths[i]);
      } catch (Exception e) {
        System.out.println("ERROR LOADING PATH " + Constants.Autonomous.paths[i] + ":" + e);
      }
    }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // COMPETITION CONTROLS
    // Driver

    OI.driverViewButton.whileTrue(new ZeroAngleMidMatch(drive)); // zero pidgeon

    OI.driverRT.whileTrue(new SetIntake(intake, 0.3));
    OI.driverLT.whileTrue(new SetIntake(intake, -0.3));

    OI.driverY.whileTrue(new SetElevatorPercent(elevator, 0.3));
    OI.driverA.whileTrue(new SetElevatorPercent(elevator, -0.3));

    OI.driverX.whileTrue(new SetPivotState(pivot, PivotState.DEFAULT));
    OI.driverB.whileTrue(new SetPivotState(pivot, PivotState.GROUND_CORAL));

    OI.driverX.whileTrue(new SetPivotPercent(pivot, 0.3));
    OI.driverB.whileTrue(new SetPivotPercent(pivot, -0.3));

    OI.driverRB.whileTrue(new SetTwistState(twist, TwistState.UP));
    OI.driverLB.whileTrue(new SetTwistState(twist, TwistState.SIDE));

    OI.driverRB.whileTrue(new SetTwistPercent(twist, 0.3));
    OI.driverLB.whileTrue(new SetTwistPercent(twist, -0.3));

    OI.driverMenuButton.whileTrue(new SetRobotPose(drive, 10.375, 1.5, 0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    int selectedPath = Constants.Autonomous.getSelectedPathIndex();
    if (selectedPath >= Constants.Autonomous.paths.length) {
      selectedPath = -1;
    }
    if (selectedPath == -1) {
      System.out.println("Selected Path: None");
      return new DoNothing();
    } else {
      this.drive.autoInit(autoPoints[selectedPath]);
      System.out.println("Selected Path: " + Constants.Autonomous.paths[selectedPath]);
      return this.autos[selectedPath];
    }
  }
}
