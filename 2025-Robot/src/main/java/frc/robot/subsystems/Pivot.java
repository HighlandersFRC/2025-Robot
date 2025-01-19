// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Pivot.PivotState;

public class Pivot extends SubsystemBase {
  /** Creates a new Pivot. */
  public Pivot() {}

  public enum PivotState {
    L1,
    L23,
    L4,
    PROCESSOR,
    NET,
    FEEDER,
    GROUND_CORAL,
    GROUND_ALGAE,
    REEF_ALGAE,
    DEFAULT,
    SCORE,
  }
  
  private PivotState wantedState = PivotState.DEFAULT;
  private PivotState systemState = PivotState.DEFAULT;

  public void setWantedState(PivotState wantedState) {
    this.wantedState = wantedState;
  }

  private PivotState handleStateTransition() {
  switch (wantedState) {
    case DEFAULT:
      return PivotState.DEFAULT;
    case L1:
      return PivotState.L1;
    case L23:
      return PivotState.L23;
    case L4:
      return PivotState.L4;
    case FEEDER:
      return PivotState.FEEDER;
    case REEF_ALGAE:
      return PivotState.REEF_ALGAE;
    case GROUND_CORAL:
      return PivotState.GROUND_CORAL;
    case GROUND_ALGAE:
      return PivotState.GROUND_ALGAE;
    case PROCESSOR:
      return PivotState.PROCESSOR;
    case NET:
      return PivotState.NET;
    case SCORE:
      return PivotState.SCORE;
    default:
      return PivotState.DEFAULT;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
