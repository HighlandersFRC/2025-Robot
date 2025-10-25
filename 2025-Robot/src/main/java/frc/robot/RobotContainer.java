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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoCoralGroundPickupFollower;
import frc.robot.commands.AutoPlaceL2Follower;
import frc.robot.commands.AutoPlaceL4Follower;
import frc.robot.commands.DoNothing;
import frc.robot.commands.FeederPickup;
import frc.robot.commands.FeederPickupFollower;
import frc.robot.commands.FullSendFollower;
import frc.robot.commands.PolarAutoFollower;
import frc.robot.commands.SetAlgaeMode;
import frc.robot.commands.SetClimberPivotTorque;
import frc.robot.commands.SetRobotState;
import frc.robot.commands.SetRobotStateComplicated;
import frc.robot.commands.SetRobotStateComplicatedContinuous;
import frc.robot.commands.SetRobotStateOnce;
import frc.robot.commands.SetRobotStateSimple;
import frc.robot.commands.SetRobotStateSimpleOnce;
import frc.robot.commands.ZeroAngleMidMatch;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Twist;
import frc.robot.subsystems.Superstructure.SuperState;

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
        Elevator elevator = new Elevator();
        Drive drive = new Drive(peripherals, elevator);
        Manipulator manipulator = new Manipulator();
        Lights lights = new Lights();
        Pivot pivot = new Pivot();
        Twist twist = new Twist();
        Climber climber = new Climber();
        Intake intake = new Intake();
        Superstructure superstructure = new Superstructure(drive, elevator, manipulator, pivot, twist, climber, lights,
                        peripherals, intake);

        public boolean algaeMode = false;
        boolean manualMode = false;
        boolean yPressed = false;
        RobotContainer m_container = this;

        HashMap<String, Supplier<Command>> commandMap = new HashMap<String, Supplier<Command>>() {
                {
                        // put("AutoPlaceL2", () -> new AutoPlaceL2Follower(superstructure, drive, 3.3));
                        // put("AutoPlaceL4", () -> new AutoPlaceL4Follower(superstructure, drive, 3.3));
                        // put("AutoFeeder", () -> new FeederPickupFollower(superstructure, drive));
                        // put("FeederIntake", () -> new FeederPickup(superstructure));
                        // put("Outake", () -> new SetRobotStateSimple(superstructure, SuperState.OUTAKE));
                        // put("L1", () -> new SetRobotStateSimple(superstructure, SuperState.AUTO_L1_PLACE));
                        // put("Idle", () -> new SetRobotStateSimple(superstructure, SuperState.IDLE));
                        // put("Full Send", () -> new FullSendFollower(drive, null, false));
                        // put("IntakeLollipop", () -> new SetRobotState(superstructure, SuperState.LOLLIOP_PICKUP));
                        // put("Net", () -> new SetRobotStateSimple(superstructure, SuperState.NET));
                        // put("GroundIntake", () -> new SetRobotStateComplicatedContinuous(superstructure,
                        //                 SuperState.GROUND_CORAL_PICKUP_FRONT, SuperState.PASSOFF_IDLE));
                        // put("ReefAlgaeL2", () -> new SetRobotState(superstructure, SuperState.L2_ALGAE_PICKUP));
                        // put("ReefAlgaeL3", () -> new SetRobotState(superstructure, SuperState.L3_ALGAE_PICKUP));
                        // // put("ReefAlgae", () -> new ReefAlgaePickupFollower(superstructure, drive,
                        // // 5.0, m_container));
                        // put("AutoIntake", () -> new AutoCoralGroundPickupFollower(superstructure, drive, 4.0));
                        // put("PassoffOutakeIdle", () -> new SetRobotStateSimpleOnce(superstructure,
                        //                 SuperState.PASSOFF_OUTAKE_IDLE));
                        // put("ToggleAlgaeMode", () -> new SetAlgaeMode(m_container));
                }
        };

        File[] autoFiles = new File[Constants.Autonomous.paths.length];
        Command[] autos = new Command[Constants.Autonomous.paths.length];
        JSONObject[] autoJSONs = new JSONObject[Constants.Autonomous.paths.length];
        JSONArray[] autoPoints = new JSONArray[Constants.Autonomous.paths.length];

        // HashMap<String, BooleanSupplier> conditionMap = new HashMap<String, BooleanSupplier>() {
        //         {
        //                 put("HasCoral", () -> manipulator.hasCoralSticky() || intake.hasCoral());
        //         }
        // };

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the trigger bindings
                configureBindings();
                // Load the Path Files
                // for (int i = 0; i < Constants.Autonomous.paths.length; i++) {
                //         try {
                //                 autoFiles[i] = new File(
                //                                 Filesystem.getDeployDirectory().getPath() + "/"
                //                                                 + Constants.Autonomous.paths[i]);
                //                 FileReader scanner = new FileReader(autoFiles[i]);
                //                 autoJSONs[i] = new JSONObject(new JSONTokener(scanner));
                //                 autoPoints[i] = (JSONArray) autoJSONs[i].getJSONArray("paths").getJSONObject(0)
                //                                 .getJSONArray("sampled_points");
                //                 autos[i] = new PolarAutoFollower(autoJSONs[i], drive, lights, peripherals, commandMap,
                //                                 conditionMap);
                //                 java.util.logging.Logger.getGlobal()
                //                                 .info("Loaded Path: " + Constants.Autonomous.paths[i]);
                //         } catch (Exception e) {
                //                 java.util.logging.Logger.getGlobal()
                //                                 .severe("ERROR LOADING PATH " + Constants.Autonomous.paths[i] + ":"
                //                                                 + e);
                //         }
                // }
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
                OI.driverViewButton.whileTrue(new ZeroAngleMidMatch(drive)); // zero pidgeon
                OI.driverMenuButton.whileTrue(new SetRobotStateSimple(superstructure, SuperState.DEFAULT)); // return to default state
                OI.driverX.whileTrue(new SetRobotState(superstructure, SuperState.GROUND_INTAKE_DOWN));
                OI.driverB.whileTrue(new SetRobotState(superstructure, SuperState.GROUND_INTAKE_UP));
                OI.driverY.whileTrue(new SetRobotStateOnce(superstructure, SuperState.CLIMBER_DOWN));
                OI.driverA.whileTrue(new SetRobotStateOnce(superstructure, SuperState.CLIMBER_UP));
                // OI.driverPOVUp.whileTrue(new SetRobotStateSimple(superstructure, L1));
                // OI.driverPOVLeft.whileTrue(new SetRobotStateSimple(superstructure, L2));
                // OI.driverPOVDown.whileTrue(new SetRobotStateSimple(superstructure, L3));
                // OI.driverPOVRight.whileTrue(new SetRobotStateSimple(superstructure, L4));
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
                        java.util.logging.Logger.getGlobal().info("Selected Path: None");
                        return new DoNothing();
                } else {
                        this.drive.autoInit(autoPoints[selectedPath]);
                        java.util.logging.Logger.getGlobal()
                                        .info("Selected Path: " + Constants.Autonomous.paths[selectedPath]);
                        return this.autos[selectedPath];
                }
        }
}
