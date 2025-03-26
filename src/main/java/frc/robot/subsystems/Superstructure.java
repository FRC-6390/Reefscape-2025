// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import ca.frc6390.athena.commands.RunnableTrigger;
import ca.frc6390.athena.controllers.DelayedOutput;
import ca.frc6390.athena.mechanisms.StateMachine;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.Elevator.ElevatorState;
import frc.robot.Constants.EndEffector.ArmState;
import frc.robot.Constants.EndEffector.WristState;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.EndEffector;
import frc.robot.subsystems.superstructure.EndEffector.EndEffectorState;
import frc.robot.subsystems.superstructure.EndEffector.EndEffectorTuple;

public class Superstructure extends SubsystemBase {
  
  private final StateMachine<Double, ElevatorState> elevatorStateMachine;
  private final StateMachine<EndEffectorTuple, EndEffectorState> endEffectorStateMachine;

  public final StateMachine<SuperstructureTuple, SuperstructureState> stateMachine;

  public Elevator elevator;
  private final EndEffector endEffector;

  private final RunnableTrigger autoDropElevatorTrigger;
  private final RunnableTrigger liftIntake;
  private final DelayedOutput atL4;
  private final DelayedOutput atL3;
  private final DelayedOutput atL2;
  private final DelayedOutput atL1;

  private final DelayedOutput atHome;
  private final DelayedOutput elevatorAtIntake;

  private boolean autoDropElevator = true;
  private boolean endEffectorEnabled = true;
  private boolean elevatorEnabled = true;

  private SuperstructureState prevState;

  public record SuperstructureTuple(EndEffectorState endEffector,  ElevatorState elevator) {}
    
    public enum SuperstructureState implements SetpointProvider<SuperstructureTuple>
    {
        AlgaeHigh(new SuperstructureTuple(EndEffectorState.AlgaeHigh, ElevatorState.AlgaeHigh)),
        NONE(new SuperstructureTuple(null,null)),

        AlgaeLow(new SuperstructureTuple(EndEffectorState.AlgaeLow, ElevatorState.AlgaeLow)),
        AlgaeScore(new SuperstructureTuple(EndEffectorState.AlgaeScore, ElevatorState.L4)),
        ScoreAlgae(new SuperstructureTuple(EndEffectorState.ScoreAlgae, null)),

        AlgaeSpit(new SuperstructureTuple(EndEffectorState.AlgaeScore, null)),

        L4(new SuperstructureTuple(EndEffectorState.L4, ElevatorState.L4)),
        L3(new SuperstructureTuple(EndEffectorState.L3, ElevatorState.L3)),
        L2(new SuperstructureTuple(EndEffectorState.L2, ElevatorState.L2)),
        L1(new SuperstructureTuple(EndEffectorState.L1, ElevatorState.L1)),
        StartConfiguration(new SuperstructureTuple(EndEffectorState.StartConfiguration, ElevatorState.HomeReset)),
        Align(new SuperstructureTuple(EndEffectorState.Home, ElevatorState.Aligning)),
        Home(new SuperstructureTuple(EndEffectorState.Home, ElevatorState.HomeReset)),
        Stopped(new SuperstructureTuple(EndEffectorState.Stop, null)),

        HomePID(new SuperstructureTuple(EndEffectorState.Home, ElevatorState.HomePID)),

        Score(new SuperstructureTuple(EndEffectorState.Score, null)),
        Intaking(new SuperstructureTuple(EndEffectorState.Intaking,  ElevatorState.Intaking));
        

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

  public Superstructure(Elevator elevator, EndEffector endEffector) 
  {
    this.endEffector = endEffector;
    this.elevator = elevator;
    this.elevatorStateMachine = elevator.getStateMachine();
    this.endEffectorStateMachine = endEffector.getStateMachine();
    this.stateMachine = new StateMachine<Superstructure.SuperstructureTuple,Superstructure.SuperstructureState>(SuperstructureState.Home, () -> elevatorStateMachine.atGoalState() && endEffectorStateMachine.atGoalState());
    this.autoDropElevatorTrigger = new RunnableTrigger(() -> autoDropElevator && endEffector.hasNoPiece() && endEffector.isScoring() && elevatorStateMachine.atState(ElevatorState.L1,ElevatorState.L2,ElevatorState.L3,ElevatorState.L4) && !endEffectorStateMachine.getGoalState().equals(EndEffectorState.ScoreAlgae));
    this.liftIntake = new RunnableTrigger(() -> endEffector.hasGamePiece());  
    this.atL4 = new DelayedOutput(() -> endeffectorAtState(EndEffectorState.L4), 0.125);
    this.atL3 = new DelayedOutput(() -> endeffectorAtState(EndEffectorState.L3), 0.125);
    this.atL2 = new DelayedOutput(() -> endeffectorAtState(EndEffectorState.L2), 0.125);
    this.atL1 = new DelayedOutput(() -> endeffectorAtState(EndEffectorState.L1), 0.125);

    this.atHome = new DelayedOutput(() -> endeffectorAtState(EndEffectorState.Home), 0.125);
    this.elevatorAtIntake = new DelayedOutput(() -> elevatorStateMachine.atState(ElevatorState.Intaking), 0.125);

    autoDropElevatorTrigger.onTrue(() -> {setSuper(SuperstructureState.HomePID);});  
    liftIntake.onTrue(setState(SuperstructureState.Align));

    prevState = SuperstructureState.Home;
  }

  public boolean l4Ready()
  {
   return atL4.getAsBoolean();
  }

  public boolean l3Ready()
  {
   return atL3.getAsBoolean();
  }
  public boolean l2Ready()
  {
   return atL2.getAsBoolean();
  }
  public boolean l1Ready()
  {
   return atL1.getAsBoolean();
  }

  public Superstructure setAutoDropElevator(boolean autoDropElevator) {
      this.autoDropElevator = autoDropElevator;
      return this;
  }

  public Command WaitForElevator() {
    return elevatorStateMachine.waitUntilAtGoal();
  }

  public Command WaitForEjector() {
    return new WaitUntilCommand(() -> !endEffector.hasGamePiece());
  }

  public Command WaitForL4() 
  { 
    return endEffectorStateMachine.waitUntil(EndEffectorState.L4);
    // return new WaitUntilCommand(() -> atL4.getAsBoolean());
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
    return new InstantCommand(() -> stateMachine.queueState(state));
  }
  public void setSuper(SuperstructureState state) {
    stateMachine.queueState(state);
  }

  public boolean drop()
  {
   return autoDropElevator && endEffector.hasNoPiece() && endEffector.isScoring() && elevatorStateMachine.atState(ElevatorState.L1,ElevatorState.L2,ElevatorState.L3,ElevatorState.L4) && !endEffectorStateMachine.getGoalState().equals(EndEffectorState.ScoreAlgae); 
  }
  public InstantCommand setElevator(ElevatorState state){
    return new InstantCommand(() -> elevatorStateManager(state));
  }

  public InstantCommand setEndEffector(EndEffectorState state){
    return new InstantCommand(() -> endEffectorStateManager(state));
  }

  public void elevatorStateManager(ElevatorState state){
    if(elevatorEnabled){
      // if(elevatorStateMachine.getGoalState().equals(Elevator.ElevatorState.Home) && elevatorStateMachine.atGoalState())
      // {
      //   elevatorStateManager(ElevatorState.Aligning);
      // }

      switch (state) {
        case L1,L2,L3,L4,HomeReset,HomePID,Intaking:
        if(prevState.states.elevator != null)
        {
        if((prevState.states.elevator.equals(ElevatorState.L4)||prevState.states.elevator.equals(ElevatorState.L3)||prevState.states.elevator.equals(ElevatorState.L2)||prevState.states.elevator.equals(ElevatorState.L1)))
        {
          endEffectorStateMachine.queueState(EndEffectorState.Home);
          elevatorStateMachine.queueState(prevState.states.elevator, () -> atHome.getAsBoolean());  
          if(state.equals(ElevatorState.L4))
          {
            endEffectorStateMachine.queueState(EndEffectorState.L4, () -> atHome.getAsBoolean());
          }
          if(state.equals(ElevatorState.L3))
          {
            endEffectorStateMachine.queueState(EndEffectorState.L3, () -> atHome.getAsBoolean());
          }
          if(state.equals(ElevatorState.L2))
          {
            endEffectorStateMachine.queueState(EndEffectorState.L2, () -> atHome.getAsBoolean());
          }
          if(state.equals(ElevatorState.L1))
          {
            endEffectorStateMachine.queueState(EndEffectorState.L1, () -> atHome.getAsBoolean());
          }
          if(state.equals(ElevatorState.Intaking))
          {
            endEffectorStateMachine.queueState(EndEffectorState.Intaking, () -> atHome.getAsBoolean() && elevatorAtIntake.getAsBoolean());
          }
        }
        }
        default:
        elevatorStateMachine.queueState(state);
          return;
      }
    }
  }

  public void endEffectorStateManager(EndEffectorState state){

    if(endEffectorEnabled){
      switch (state) {

        case L1, L2, L3, L4, AlgaeScore, AlgaeHigh, AlgaeLow:
          // if(!elevatorStateMachine.atGoalState())
          // {
          if(!prevState.states.endEffector.equals(EndEffectorState.AlgaeScore))
          {
            endEffectorStateMachine.queueState(EndEffectorState.Home);
          }
          // }
          // else
          // {
       
          endEffectorStateMachine.queueState(state, () -> elevatorStateMachine.atGoalState());
          
          // }
          break;
        case Home:
          if(prevState.equals(SuperstructureState.Intaking))
          {
          endEffectorStateMachine.queueState(EndEffectorState.Transition);
          endEffectorStateMachine.queueState(EndEffectorState.Home, () -> endEffector.getJoint1().getStateMachine().atState(ArmState.TransitionState) && endEffector.getJoint2().getStateMachine().atState(WristState.TransitionState));
          }
          else
          {
          endEffectorStateMachine.queueState(state);
          }
          break;
        default:
        endEffectorStateMachine.queueState(state);
          return;
      }
    }
  }

  public boolean endeffectorAtState(EndEffectorState state)
  {
    return endEffectorStateMachine.atState(state);
  }

  public void update(){
    elevatorStateMachine.update();
    endEffectorStateMachine.update();
    stateMachine.update();

    SmartDashboard.putBoolean("AtGoalState", elevatorStateMachine.atGoalState());

    SuperstructureState state = stateMachine.getGoalState();
    SuperstructureTuple val = stateMachine.getGoalStateSetpoint();

    if(!prevState.equals(state)){
      switch (state) {
        default:
          if (val.elevator != null)elevatorStateManager(val.elevator);
          if (val.endEffector != null) endEffectorStateManager(val.endEffector);
          break;
      }
    }

    // if(endEffector.isAutoEndScoring() && endEffector.isScoring() && endEffector.hasNoPiece()) {
    //   stateMachine.queueState(SuperstructureState.Home);
    // }

    prevState = state;
  }

  @Override
  public void periodic() {
    update();
  }
  
  
}
