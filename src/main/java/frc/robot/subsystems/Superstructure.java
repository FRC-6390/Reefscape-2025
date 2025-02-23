// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import ca.frc6390.athena.mechanisms.StateMachine;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.superstructure.Climber;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.EndEffector;
import frc.robot.subsystems.superstructure.EndEffector.AlgaeExtensionState;

public class Superstructure {
  
  /** Creates a new Superstructure. */
  StateMachine<Elevator.ElevatorState> elevator;
  StateMachine<Climber.ClimberState> climber;
  StateMachine<EndEffector.AlgaeExtensionState> algaeMachine;
  StateMachine<EndEffector.EndEffectorState> endEffector;
  public EndEffector effector;

  public Superstructure(Climber climber, Elevator elevator, EndEffector effector) 
  {
    this.climber = climber.getStateMachine();
    this.effector= effector;
    this.elevator = elevator.getStateMachine();
    this.endEffector = effector.getStateMachine();
    this.algaeMachine = effector.getAlgaeStateMachine();
  }

  public InstantCommand setClimber(Climber.ClimberState state){
    return new InstantCommand(() -> climberStateManager(state));
  }

  public InstantCommand setElevator(Elevator.ElevatorState state){
    return new InstantCommand(() -> elevatorStateManager(state));
  }

  public InstantCommand setEndEffectir(EndEffector.EndEffectorState state){
    return new InstantCommand(() -> endEffectorStateManager(state));
  }

  public InstantCommand setAlgaeMachine(EndEffector.AlgaeExtensionState state){
    return new InstantCommand(() -> algaeStateManager(state));
  }

  public void climberStateManager(Climber.ClimberState state){
    switch (state) {
      case Climb:
        elevatorStateManager(Elevator.ElevatorState.L2);
        endEffectorStateManager(EndEffector.EndEffectorState.Home);
        climber.setGoalState(state, () -> elevator.atState(Elevator.ElevatorState.L2) && endEffector.atState(EndEffector.EndEffectorState.Home));
        break;
      case Home:
        climber.setGoalState(state);
        break;
    }
  }

  public void algaeStateManager(AlgaeExtensionState state)
  {
    algaeMachine.setGoalState(state);
  }

  public void elevatorStateManager(Elevator.ElevatorState state){
    switch (state) {
      case Home, Feeder, StartConfiguration:
        climberStateManager(Climber.ClimberState.Home);
        endEffectorStateManager(EndEffector.EndEffectorState.Home);
        elevator.setGoalState(state, () -> !climber.atState(Climber.ClimberState.Climb) && endEffector.atState(EndEffector.EndEffectorState.Home));
        break;
      case L1:
        climberStateManager(Climber.ClimberState.Home);
        elevator.setGoalState(state, () -> !climber.atState(Climber.ClimberState.Climb));
        break;
      case L2, L3, L4:
        elevator.setGoalState(state);
    }
  }

  public void endEffectorStateManager(EndEffector.EndEffectorState state){
    switch (state) {
      case StartConfiguration:
        elevatorStateManager(Elevator.ElevatorState.L1);
        endEffector.setGoalState(state, () -> elevator.atState(Elevator.ElevatorState.L1));
      case Home:
        endEffector.setGoalState(state);
        break;
      case Left, LeftL4, Right, RightL4:
        endEffector.setGoalState(state, () -> !elevator.atState(Elevator.ElevatorState.Home));
        break;
    }
  }
  
  public void ejectPiece(double speed){
    effector.setRollers(speed);
  }
}
