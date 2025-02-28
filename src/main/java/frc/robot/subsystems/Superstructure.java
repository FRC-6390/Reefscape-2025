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
import frc.robot.subsystems.superstructure.Elevator.ElevatorState;
import frc.robot.subsystems.superstructure.EndEffector.EndEffectorState;
import frc.robot.subsystems.superstructure.EndEffector.EndEffectorTuple;
import frc.robot.utils.ReefScoringPos.ReefPole;

public class Superstructure extends SubsystemBase {
  
  private final StateMachine<Double, ElevatorState> elevatorStateMachine;
  private final StateMachine<EndEffectorTuple, EndEffectorState> endEffectorStateMachine;

  private final StateMachine<SuperstructureTuple, SuperstructureState> stateMachine;

  private final RobotBase<?> base;

  private double dist;
  private final RunnableTrigger autoDropElevatorTrigger;
  private boolean autoDropElevator = true;
  private boolean endEffectorEnabled = true;
  private boolean elevatorEnabled = true;

  public record SuperstructureTuple(EndEffectorState endEffector,  ElevatorState elevator) {}
    
    public enum SuperstructureState implements SetpointProvider<SuperstructureTuple>
    {
        AlgaeHigh(new SuperstructureTuple(EndEffectorState.AlgaeHigh, ElevatorState.AlgaeHigh)),
        AlgaeLow(new SuperstructureTuple(EndEffectorState.AlgaeLow, ElevatorState.AlgaeLow)),
        L4(new SuperstructureTuple(EndEffectorState.L4, ElevatorState.L4)),
        L3(new SuperstructureTuple(EndEffectorState.L3, ElevatorState.L3)),
        L2(new SuperstructureTuple(EndEffectorState.L2, ElevatorState.L2)),
        L1(new SuperstructureTuple(EndEffectorState.L1, ElevatorState.L1)),
        Home(new SuperstructureTuple(EndEffectorState.Home, ElevatorState.Home)),
        Score(new SuperstructureTuple(EndEffectorState.Score, null));


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


  public Superstructure(Elevator elevator, EndEffector endEffector, RobotBase<?> base) 
  {
    this.elevatorStateMachine = elevator.getStateMachine();
    this.endEffectorStateMachine = endEffector.getStateMachine();
    this.base = base;
    this.stateMachine = new StateMachine<Superstructure.SuperstructureTuple,Superstructure.SuperstructureState>(SuperstructureState.Home, () -> elevatorStateMachine.atGoalState() && endEffectorStateMachine.atGoalState());
    this.autoDropElevatorTrigger = new RunnableTrigger(() -> autoDropElevator && endEffector.isScoring() && elevatorStateMachine.atAnyState(ElevatorState.L1,ElevatorState.L2,ElevatorState.L3,ElevatorState.L4));
    autoDropElevatorTrigger.onFalse(setState(SuperstructureState.Home));

    dist = 9999;
  }

  public boolean closeEnough()
  {

    LimeLight ll = base.getCameraFacing(ReefPole.getCenterReef());
    SmartDashboard.putNumber("Dist", dist);
    if(ll != null)
    {
      ReefPole pole = ReefPole.getPoleFromID(ll.getAprilTagID(), ll);
      if(pole != null && ll.hasValidTarget())
      {
        dist = ll.getTargetHorizontalOffset();
      }
    }

    if(Math.abs(dist) < 15)
    {
      return true;
    }
    else{
      return false;
    } 
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
        elevatorStateMachine.setGoalState(state, () -> endEffectorStateMachine.atState(EndEffectorState.Home));
        break;
      case L1, AlgaeHigh, AlgaeLow:
        elevatorStateMachine.setGoalState(state);
        break;
      case L2, L3, L4:
        elevatorStateMachine.setGoalState(state);
    }
  }

  public void endEffectorStateManager(EndEffector.EndEffectorState state){
    endEffectorStateMachine.setGoalState(state);
  }

  public Translation2d getCage()
  {
    Alliance team = DriverStation.getAlliance().get();
    if(team.equals(Alliance.Red))
    {
      return new Translation2d(8.775, 6.201);
    }
    else
    {
      return new Translation2d(8.787, 1.921);
    }
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
      case AlgaeLow:
      case Home:
      case Score:
        if(val.elevator != null && elevatorEnabled) elevatorStateManager(val.elevator);
        if(val.endEffector != null && endEffectorEnabled) endEffectorStateManager(val.endEffector);
        break;
      default:
        break;
    }
  }

  @Override
  public void periodic() {
    update();
  }
  
  
}
