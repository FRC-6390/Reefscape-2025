// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.mechanisms.StateMachine;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.EndEffector;
import frc.robot.subsystems.superstructure.Elevator.ElevatorState;
import frc.robot.subsystems.superstructure.EndEffector.EndEffectorState;
import frc.robot.subsystems.superstructure.EndEffector.EndEffectorTuple;
import frc.robot.utils.ReefScoringPos.ReefPole;

public class Superstructure {
  
  /** Creates a new Superstructure. */
  private final StateMachine<Double, ElevatorState> elevator;
  private final StateMachine<EndEffectorTuple, EndEffectorState> endEffector;

  private final RobotBase<?> base;

  private double dist;
  private Translation2d translation;

  public record SuperstructureTuple(EndEffectorState endEffector,  ElevatorState elevator) {}
    
    public enum SuperstructureState implements SetpointProvider<SuperstructureTuple>
    {
        AlgaeHigh(new SuperstructureTuple(EndEffectorState.AlgaeHigh, ElevatorState.AlgaeHigh)),
        AlgaeLow(new SuperstructureTuple(EndEffectorState.AlgaeLow, ElevatorState.AlgaeLow)),
        L4(new SuperstructureTuple(EndEffectorState.L4, ElevatorState.L4)),
        L3(new SuperstructureTuple(EndEffectorState.L3, ElevatorState.L3)),
        L2(new SuperstructureTuple(EndEffectorState.L2, ElevatorState.L2)),
        L1(new SuperstructureTuple(EndEffectorState.L1, ElevatorState.L1)),
        Home(new SuperstructureTuple(EndEffectorState.Home, ElevatorState.Home));


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
    this.elevator = elevator.getStateMachine();
    this.endEffector = endEffector.getStateMachine();
    this.base = base;
    
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



  public InstantCommand setElevator(Elevator.ElevatorState state){
    return new InstantCommand(() -> elevatorStateManager(state));
  }

  public InstantCommand setEndEffector(EndEffectorState state){
    return new InstantCommand(() -> endEffectorStateManager(state));
  }

  public void elevatorStateManager(Elevator.ElevatorState state){
    switch (state) {
      case Home:
        endEffectorStateManager(EndEffectorState.Home);
        elevator.setGoalState(state, () -> endEffector.atState(EndEffectorState.Home));
        break;
      case L1, AlgaeHigh, AlgaeLow:
        elevator.setGoalState(state);
        break;
      case L2, L3, L4:
        elevator.setGoalState(state);
    }
  }

  public void returnAfterScore()
  {
    if(!effector.beamBreakCenter.getAsBoolean() && !effector.beamBreakRight.getAsBoolean() && !effector.beamBreakLeft.getAsBoolean() && effector.algStateMachine.getGoalState().equals(AlgaeExtensionState.Home))
    {
      elevatorStateManager(ElevatorState.Home);
    }
  }
  public void endEffectorStateManager(EndEffector.EndEffectorState state){
    endEffector.setGoalState(state);
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
  
  
}
