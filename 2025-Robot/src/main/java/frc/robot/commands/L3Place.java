// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class L3Place extends SequentialCommandGroup {
  /** Creates a new L3Coral. */
  Elevator elevator;
  Intake intake;
  Superstructure superstructure;

  public L3Place(Elevator elevator, Intake intake, Superstructure superstructure) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelRaceGroup(
            new SetRobotState(superstructure, SuperState.IDLE),
            new WaitCommand(0.2)),
        new ParallelRaceGroup(
            new SetIntake(intake, 0.3),
            new SetRobotState(superstructure, SuperState.ELEVATOR_OFF)));
  }
}
