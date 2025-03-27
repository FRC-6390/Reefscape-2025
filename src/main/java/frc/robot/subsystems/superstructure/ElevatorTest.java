// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import ca.frc6390.athena.commands.RunnableTrigger;
import ca.frc6390.athena.mechanisms.ElevatorMechanism.StatefulElevatorMechanism;
import ca.frc6390.athena.mechanisms.MechanismConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import frc.robot.Constants.Elevator.ElevatorState;

public class ElevatorTest extends StatefulElevatorMechanism<ElevatorState>{
  
  private final RunnableTrigger idle;
  
  public ElevatorTest(MechanismConfig<StatefulElevatorMechanism<ElevatorState>> config, ElevatorFeedforward feedforward,ElevatorState initialState) {
    super(config, feedforward, initialState);
    idle = new RunnableTrigger(() -> getLimitSwitches()[0].getAsBoolean() && !getStateMachine().isGoalState(ElevatorState.Intaking));
    idle.onTrue(() -> getStateMachine().queueState(ElevatorState.Aligning));
  }

}