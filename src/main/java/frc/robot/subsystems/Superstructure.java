// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import ca.frc6390.athena.mechanisms.StateMachine;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.superstructure.Climber;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.EndEffector;

public class Superstructure {
  
  /** Creates a new Superstructure. */
  StateMachine<Elevator.State> elevator;
  StateMachine<Climber.State> climber;
  StateMachine<EndEffector.State> endEffector;

  public Superstructure(Climber climber) 
  {
    this.climber = climber.getStateMachine();
  }

  public InstantCommand setClimber(Climber.State state){
    return new InstantCommand(() -> climberStateManager(state));
  }

  public InstantCommand setElevator(Elevator.State state){
    return new InstantCommand(() -> elevatorStateManager(state));
  }

  public InstantCommand setEndEffectir(EndEffector.State state){
    return new InstantCommand(() -> endEffectorStateManager(state));
  }

  public void climberStateManager(Climber.State state){
    switch (state) {
      case Climb:
        elevatorStateManager(Elevator.State.L2);
        endEffectorStateManager(EndEffector.State.Home);
        climber.setGoalState(state, () -> elevator.atState(Elevator.State.L2) && endEffector.atState(EndEffector.State.Home));
        break;
      case Home:
        climber.setGoalState(state);
        break;
    }
    FeedF
  }

  public void elevatorStateManager(Elevator.State state){
    switch (state) {
      case Home, Feeder, StartConfiguration:
        climberStateManager(Climber.State.Home);
        endEffectorStateManager(EndEffector.State.Home);
        elevator.setGoalState(state, () -> !climber.atState(Climber.State.Climb) && endEffector.atState(EndEffector.State.Home));
        break;
      case L1:
        climberStateManager(Climber.State.Home);
        elevator.setGoalState(state, () -> !climber.atState(Climber.State.Climb));
        break;
      case L2, L3, L4:
        elevator.setGoalState(state);
    }
  }

  public void endEffectorStateManager(EndEffector.State state){
    switch (state) {
      case StartConfiguration:
        elevatorStateManager(Elevator.State.L1);
        endEffector.setGoalState(state, () -> elevator.atState(Elevator.State.L1));
      case Home:
        endEffector.setGoalState(state);
        break;
      case Left, LeftL4, Right, RightL4:
        endEffector.setGoalState(state, () -> !elevator.atState(Elevator.State.Home));
        break;
    }
  }
}
