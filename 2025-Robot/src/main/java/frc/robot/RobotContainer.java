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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DoNothing;
import frc.robot.commands.SetRobotStateOnce;
import frc.robot.commands.SetRobotStateSimple;
import frc.robot.commands.ZeroAngleMidMatch;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Peripherals;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

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
        Shooter shooter = new Shooter();
        Intake intake = new Intake();
        Feeder feeder = new Feeder();
        Superstructure superstructure = new Superstructure(drive, shooter, intake, feeder);

        public boolean algaeMode = false;
        boolean manualMode = false;
        boolean yPressed = false;
        RobotContainer m_container = this;

        HashMap<String, Supplier<Command>> commandMap = new HashMap<String, Supplier<Command>>() {
                {
                        put("Idle", () -> new SetRobotStateSimple(superstructure, SuperState.IDLE));
                }
        };

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

                OI.driverRT.whileTrue(new SetRobotStateOnce(superstructure, SuperState.INTAKE));
                OI.driverLT.whileTrue(new SetRobotStateOnce(superstructure, SuperState.OUTAKE));

                OI.driverA.whileTrue(new SetRobotStateOnce(superstructure, SuperState.PREP_SHOOT_AUTO));
                OI.driverY.whileTrue(new SetRobotStateOnce(superstructure, SuperState.PREP_SHOOT_NORMAL));

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return new DoNothing();

        }
}
