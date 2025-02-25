// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import ca.frc6390.athena.controllers.DelayedOutput;
import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.mechanisms.StateMachine;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.Elevator.ElevatorState;
import frc.robot.subsystems.superstructure.EndEffector;
import frc.robot.subsystems.superstructure.EndEffector.AlgaeExtensionState;
import frc.robot.utils.ReefScoringPos.ReefPole;

public class Superstructure {
  
  /** Creates a new Superstructure. */
  StateMachine<Elevator.ElevatorState> elevator;
  StateMachine<EndEffector.AlgaeExtensionState> algaeMachine;
  StateMachine<EndEffector.EndEffectorState> endEffector;
  public EndEffector effector;
  public RobotBase<?> base;
  public DelayedOutput output;
  public double dist;
  

  public Superstructure(Elevator elevator, EndEffector effector, RobotBase<?> base) 
  {
    this.effector= effector;
    this.elevator = elevator.getStateMachine();
    this.base = base;
    this.endEffector = effector.getStateMachine();
    this.algaeMachine = effector.getAlgaeStateMachine();
    output =  new DelayedOutput(this::closeEnough, 0.85);
    dist = 9999;
  }

  public boolean closeEnough()
  {
    LimeLight ll = base.getCameraFacing(ReefPole.A.getTranslation());
    ReefPole pole = ReefPole.getPoleFromID(ll.getAprilTagID());
    Translation2d translation = new Translation2d(9999,999);
    if(pole != null)
    {
      translation = pole.getTranslation();
    }
    dist = Math.abs(base.getLocalization().getFieldPose().getTranslation().getDistance(translation));
  
    if(dist < 0.3)
    {
      return true;
    }
    else
    {
      return false;
    }
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


  public void algaeStateManager(AlgaeExtensionState state)
  {
    algaeMachine.setGoalState(state);
  }

  public void elevatorStateManager(Elevator.ElevatorState state){
    switch (state) {
      case Home, Feeder, StartConfiguration, Climb:
        endEffectorStateManager(EndEffector.EndEffectorState.Home);
        elevator.setGoalState(state, () -> endEffector.atState(EndEffector.EndEffectorState.Home));
        break;
      case L1:
        elevator.setGoalState(state);
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
  
  public void ejectPiece(double speed)
  {
    if(output.getAsBoolean())
    {
      effector.setRollers(speed);
    }
    else
    {
      effector.setRollers(0);
    } 
  }

  public void forceEject(double speed)
  {
    effector.setRollers(speed);
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
  
  //CLIMB AUTOMATION
  public void automateClimb()
  {
    
    if(Math.abs(base.getLocalization().getFieldPose().getTranslation().getDistance(getCage()))< 2.25)
    {
      setElevator(ElevatorState.Climb);
    }
  }
}
