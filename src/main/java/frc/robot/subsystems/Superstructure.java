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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.EndEffector;
import frc.robot.subsystems.superstructure.EndEffector.AlgaeExtensionState;
import frc.robot.subsystems.superstructure.EndEffector.EjectorState;
import frc.robot.subsystems.superstructure.EndEffector.EndEffectorState;
import frc.robot.utils.ReefScoringPos.ReefPole;

public class Superstructure {
  
  /** Creates a new Superstructure. */
  StateMachine<Elevator.ElevatorState> elevator;
  StateMachine<EndEffector.AlgaeExtensionState> algaeMachine;
  StateMachine<EndEffector.EndEffectorState> endEffector;
  public Translation2d translation;
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
    LimeLight ll = base.getCameraFacing(ReefPole.getCenterReef());
    SmartDashboard.putNumber("Dist", dist);

   

    if(ll != null)
    {
    ReefPole pole = ReefPole.getPoleFromID(ll.getAprilTagID(), ll);

    if(pole != null)
    {
    translation = pole.getTranslation();
    if(translation != null)
    {
    SmartDashboard.putNumber("Translation X", pole.getTranslation().getX());
    SmartDashboard.putNumber("Translation Y", pole.getTranslation().getY());
    SmartDashboard.putNumber("ID", pole.getApriltagId());
    SmartDashboard.putNumber("LL ID", ll.getAprilTagID());
    }
    }
    if(translation != null)
    {
    dist = Math.abs(base.getLocalization().getFieldPose().getTranslation().getDistance(translation));
    } 
    if(dist < 0.2)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    return false;
  }
  }



  public InstantCommand setElevator(Elevator.ElevatorState state){
    return new InstantCommand(() -> elevatorStateManager(state));
  }

  public InstantCommand setEndEffector(EndEffector.EndEffectorState state){
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
      case Home:
        endEffectorStateManager(EndEffector.EndEffectorState.Home);
        algaeStateManager(AlgaeExtensionState.Home);
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
    endEffector.setGoalState(state);
  }
  
  public void ejectPiece(double speed)
  {
    SmartDashboard.putString("LL", base.getCameraFacing(ReefPole.getCenterReef()).config.table());
    if(base.getCameraFacing(ReefPole.getCenterReef()).config.table() == "limelight-left")
    {
      effector.ejectStateMachine.setGoalState(EjectorState.Left);
    }
    else if(base.getCameraFacing(ReefPole.getCenterReef()).config.table() == "limelight-right")
    {
      effector.ejectStateMachine.setGoalState(EjectorState.Right);
    }
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
