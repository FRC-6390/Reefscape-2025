// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Experimental;

import java.util.List;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import ca.frc6390.athena.mechanisms.ArmMechanism.StatefulArmMechanism;
import ca.frc6390.athena.mechanisms.ElevatorMechanism.StatefulElevatorMechanism;
import ca.frc6390.athena.mechanisms.SimpleMotorMechanism.StatefulSimpleMotorMechanism;
import ca.frc6390.athena.mechanisms.StateMachine;
import ca.frc6390.athena.mechanisms.StatefulMechanism;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.Superstructure.SuperstructureTuple;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;

public class SuperStructureTest<E extends Enum<E> & SetpointProvider<?>> {
    private final List<StatefulArmMechanism<?>> arms;
    private final List<StatefulElevatorMechanism<?>> elevators;
    private final List<StatefulMechanism<?>> motors;

    private final List<Constraint<E>> constraints;
    private final Map<E, E> transitions;
    private E currentState;

    public SuperStructureTest(
        List<StatefulArmMechanism<?>> arms,
        List<StatefulElevatorMechanism<?>> elevators,
        List<StatefulMechanism<?>> motors,

        List<Constraint<E>> constraints,
        Map<E, E> transitions
    ) {
        this.arms = arms;
        this.elevators = elevators;
        this.motors = motors;
        this.constraints = constraints;
        this.transitions = transitions;
    }

    // public void requestState(E desiredState) 
    // {
    //      E transition = transitions.getOrDefault(desiredState, desiredState); 
    //      boolean valid = constraints.stream().allMatch(c -> c.isValid(transition)); 
    //      if (valid) 
    //      { 
    //         this.currentState = transition; 
    //         arms.forEach(m -> m.getStateMachine().queueState(transition)); 
    //     } 
    //     else 
    //     { 
    //         System.out.println("Constraint failed: cannot move to " + transition); 
    //     }  
    // }


    public void requestState2(List<Enum<?>> stateList) 
    {


        for (Enum<?> state : stateList) 
        {
            for (StatefulArmMechanism<?> arm : arms) {
                if(arm.getStateMachine().getGoalState().getClass().isInstance(state))
                {
                    ((StateMachine) arm.getStateMachine()).queueState(state);
                }
            }

             for (StatefulElevatorMechanism<?> elevator : elevators) {
                if(elevator.getStateMachine().getGoalState().getClass().isInstance(state))
                {
                    ((StateMachine) elevator.getStateMachine()).queueState(state);
                }
            }

             for (StatefulMechanism<?> motor : motors) {
                if(motor.getStateMachine().getGoalState().getClass().isInstance(state))
                {
                    ((StateMachine) motor.getStateMachine()).queueState(state);
                }
            }
        }
        
    }

   

    public void update() {
        arms.forEach(StatefulArmMechanism::update);
    }
}
