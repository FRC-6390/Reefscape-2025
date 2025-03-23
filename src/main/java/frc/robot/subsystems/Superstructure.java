// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import ca.frc6390.athena.commands.RunnableTrigger;
import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.mechanisms.StateMachine;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.EndEffector;
import frc.robot.subsystems.superstructure.EndEffectorV2;
import frc.robot.subsystems.superstructure.Elevator.ElevatorState;
import frc.robot.subsystems.superstructure.EndEffectorV2.EndEffectorState;
import frc.robot.subsystems.superstructure.EndEffectorV2.EndEffectorTuple;
import frc.robot.utils.ReefScoringPos.ReefPole;

public class Superstructure extends SubsystemBase {
  
  private final StateMachine<Double, ElevatorState> elevatorStateMachine;
  private final StateMachine<EndEffectorTuple, EndEffectorState> endEffectorStateMachine;

  private final StateMachine<SuperstructureTuple, SuperstructureState> stateMachine;

  public Elevator elevator;
  private final RobotBase<?> base;
  private final EndEffectorV2 endEffector;
 

  private final RunnableTrigger autoDropElevatorTrigger;
  private final RunnableTrigger liftIntake;
  private final RunnableTrigger dropIntake;
 
  private boolean autoDropElevator = true;
  private boolean endEffectorEnabled = true;
  private boolean elevatorEnabled = true;

  public record SuperstructureTuple(EndEffectorState endEffector,  ElevatorState elevator) {}
    
    public enum SuperstructureState implements SetpointProvider<SuperstructureTuple>
    {
        AlgaeHigh(new SuperstructureTuple(null, ElevatorState.AlgaeHigh)),
        AlgaeRetract(new SuperstructureTuple(null, null)),
        AlgaeLow(new SuperstructureTuple(null, ElevatorState.AlgaeLow)),
        L4(new SuperstructureTuple(EndEffectorState.L4, ElevatorState.L4)),
        L3(new SuperstructureTuple(EndEffectorState.L3, ElevatorState.L3)),
        L2(new SuperstructureTuple(EndEffectorState.L2, ElevatorState.L2)),
        L1(new SuperstructureTuple(EndEffectorState.L1, ElevatorState.L1)),
        Home(new SuperstructureTuple(EndEffectorState.Home, ElevatorState.Home)),
        Score(new SuperstructureTuple(EndEffectorState.Score, null)),
        Intaking(new SuperstructureTuple(EndEffectorState.Intaking,  ElevatorState.L1));
        

        private SuperstructureTuple states;
        private SuperstructureState(SuperstructureTuple states)
        {
            this.states = states;
        }

        @Override
        public SuperstructureTuple getSetpoint()
        {
            return states;
        }
    }

  public Superstructure(Elevator elevator, EndEffectorV2 endEffector, RobotBase<?> base) 
  {
    this.endEffector = endEffector;
    this.elevator = elevator;
    this.elevatorStateMachine = elevator.getStateMachine();
    this.endEffectorStateMachine = endEffector.getStateMachine();
    this.base = base;
    this.stateMachine = new StateMachine<Superstructure.SuperstructureTuple,Superstructure.SuperstructureState>(SuperstructureState.Home, () -> elevatorStateMachine.atGoalState() && endEffectorStateMachine.atGoalState());
    this.autoDropElevatorTrigger = new RunnableTrigger(() -> autoDropElevator && endEffector.isScoring() && elevatorStateMachine.atAnyState(ElevatorState.L1,ElevatorState.L2,ElevatorState.L3,ElevatorState.L4));
    this.liftIntake = new RunnableTrigger(() -> endEffector.hasGamePiece() && stateMachine.getGoalState().equals(SuperstructureState.Home));
    this.dropIntake = new RunnableTrigger(() -> !endEffector.hasGamePiece() && stateMachine.getGoalState().equals(SuperstructureState.Home));
    autoDropElevatorTrigger.onFalse(setState(SuperstructureState.Intaking));

    
    liftIntake.onTrue(setState(SuperstructureState.Home));
    dropIntake.onTrue(setState(SuperstructureState.Intaking));

  }

  public boolean elevatorAtSetpoint()
  {
    // if(elevator.controller.atSetpoint())
    // {
    //   SmartDashboard.putBoolean("At Setpoint", elevator.controller.atSetpoint());
      return true;
    // }
    // else
    // {
      // return false;
    // }
  }


  public Superstructure setAutoDropElevator(boolean autoDropElevator) {
      this.autoDropElevator = autoDropElevator;
      return this;
  }

  public Superstructure setElevatorEnabled(boolean elevatorEnabled) {
      this.elevatorEnabled = elevatorEnabled;
      return this;
  }

  public Superstructure setEndEffectorEnabled(boolean endEffectorEnabled) {
      this.endEffectorEnabled = endEffectorEnabled;
      return this;
  }


  public InstantCommand setState(SuperstructureState state) {
    System.out.println(state.name());
    return new InstantCommand(() -> stateMachine.setGoalState(state));
  }

  public InstantCommand setElevator(Elevator.ElevatorState state){
    return new InstantCommand(() -> elevatorStateManager(state));
  }

  public InstantCommand setEndEffector(EndEffectorState state){
    return new InstantCommand(() -> endEffectorStateManager(state));
  }

  public void elevatorStateManager(Elevator.ElevatorState state){
    switch (state) {
      case Home:
        elevatorStateMachine.setGoalState(state);
        break;
      case L1, AlgaeHigh, AlgaeLow:
        elevatorStateMachine.setGoalState(state);
        break;
      case L2, L3, L4:
        elevatorStateMachine.setGoalState(state);
    }
  }

  public void endEffectorStateManager(EndEffectorState state){
    endEffectorStateMachine.setGoalState(state);
  }

  public void update(){
    elevatorStateMachine.update();
    endEffectorStateMachine.update();
    stateMachine.update();

    SuperstructureState state = stateMachine.getGoalState();
    SuperstructureTuple val = stateMachine.getGoalStateSetpoint();
    switch (state) {
      case L1:
      case L2:
      case L3:
      case L4:
      case AlgaeHigh:
      case AlgaeRetract:
      case AlgaeLow:
      case Home:
      case Score:
        if(val.elevator != null && elevatorEnabled) elevatorStateManager(val.elevator);
        if(val.endEffector != null && endEffectorEnabled) endEffectorStateManager(val.endEffector);
        break;
      default:
        break;
    }

    if(endEffector.isAutoEndScoring() && endEffector.isScoring() && !endEffector.hasGamePiece()) {
      stateMachine.setGoalState(SuperstructureState.Home);
    }

    if(!endEffector.hasGamePiece() && stateMachine.atAnyState(SuperstructureState.Home))
    {
      stateMachine.setGoalState(SuperstructureState.Intaking);
    }
    if(endEffector.hasGamePiece())
    {
      stateMachine.setGoalState(SuperstructureState.L1);
    }
  }

  @Override
  public void periodic() {
    update();
  }
  
  
}
