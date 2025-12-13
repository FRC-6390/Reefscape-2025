// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.Experimental;

import java.util.List;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.configs.DigitalInputsConfigs;

import ca.frc6390.athena.mechanisms.ArmMechanism.StatefulArmMechanism;
import ca.frc6390.athena.mechanisms.ElevatorMechanism.StatefulElevatorMechanism;
import ca.frc6390.athena.mechanisms.SimpleMotorMechanism.StatefulSimpleMotorMechanism;
import ca.frc6390.athena.mechanisms.StateMachine;
import ca.frc6390.athena.mechanisms.StatefulMechanism;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import edu.wpi.first.wpilibj.DigitalInput;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;

public class SuperStructureTest<E extends Enum<E>> {
    private final List<StatefulArmMechanism<?>> arms;
    private final List<StatefulElevatorMechanism<?>> elevators;
    private final List<StatefulMechanism<?>> motors;


    private ArrayList<Constraint<SuperStructureStates>> constraints;
    private ArrayList<ActionableConstraint<SuperStructureStates>> actionableConstraints;
    private ArrayList<Runnable> updateEvents;

    private final List<DigitalSensor> sensors;
    

    private SuperStructureStates currentState = SuperStructureStates.Home;
    private SuperStructureStates prevStates = SuperStructureStates.Home;
    private ActionableConstraint<SuperStructureStates> nextState;

    public SuperStructureTest(
        List<StatefulArmMechanism<?>> arms,
        List<StatefulElevatorMechanism<?>> elevators,
        List<StatefulMechanism<?>> motors,
        ArrayList<Runnable> updateEvents,
        ArrayList<Constraint<SuperStructureStates>> constraints,
        ArrayList<ActionableConstraint<SuperStructureStates>> actionableConstraints,

        List<DigitalSensor> sensors


    ) {
        this.arms = arms;
        this.elevators = elevators;
        this.motors = motors;
        this.constraints = constraints;
        this.updateEvents = updateEvents;
        this.actionableConstraints = actionableConstraints;
        this.sensors = sensors;
    }

    public SuperStructureStates getCurrentStates()
    {
        return currentState;
    }
    public SuperStructureStates getPrevStates()
    {
        return prevStates;
    }

    public void setGoalState(SuperStructureStates states)
    {
        Constraint<SuperStructureStates> currentConstraint = new Constraint<SuperStructureStates>(states, () -> {return true;});
        ActionableConstraint<SuperStructureStates> currentActionableConstraint = new ActionableConstraint<SuperStructureStates>(states, null, () -> {return true;});

        for (Constraint<SuperStructureStates> constraint : constraints) 
        {
         SuperStructureStates constrainedState = constraint.getTargetState();
         if(states.equals(constrainedState))   
         {
            currentConstraint = constraint;
         }
        }

        for (ActionableConstraint<SuperStructureStates> constraint : actionableConstraints) 
        {
         SuperStructureStates constrainedState = constraint.getTargetState();
         if(states.equals(constrainedState))   
         {
            currentActionableConstraint = constraint;
         }
        }
        if(currentConstraint.isValid() && currentActionableConstraint.isValid())
        {
        System.out.println("HJKHJKSHF");
        currentState = states;
        }
        else if(currentConstraint.isValid() && !currentActionableConstraint.isValid())
        {
        currentState = currentActionableConstraint.getTransition();
        nextState = currentActionableConstraint;
        }
    }

    public void addConstraint(Constraint<SuperStructureStates> constraint)
    {
        constraints.add(constraint);
    }

    public void addActionableConstraint(ActionableConstraint<SuperStructureStates> constraint)
    {
        actionableConstraints.add(constraint);
    }

    public void addUpdateEvent(Runnable runnable)
    {
        updateEvents.add(runnable);
    }

    public DigitalSensor getSensor(String name)
    {
        DigitalSensor s = null;
        for (DigitalSensor sensor : sensors) 
        {
        if(sensor.getName() == name)
        {
            s = sensor;
        }    
        }
        return s;
    }

    public boolean atState(SuperStructureStates states)
    {
        List<Enum<?>> stateList = states.getStates();
        boolean armsAt = false;
        boolean rollersAt = false;
        boolean elevatorsAt = false;
        int armCount = 0;
        int rollerCount = 0;
        int elevatorCount = 0;

        for (Enum<?> state : stateList) 
        {

            //CHECK IF ALL ARMS ARE IN POSITION
            for (StatefulArmMechanism<?> arm : arms) {
                
                if(arm.getStateMachine().getGoalState().getClass().isInstance(state))
                {
                     if (((StateMachine) arm.getStateMachine()).atState(state)) armCount++;
                }
            }
            if(armCount == arms.size()) armsAt = true;

            //CHECK IF ALL ELEVATORS ARE IN POSITION
            for (StatefulElevatorMechanism<?> elevator : elevators) {
                
                if(elevator.getStateMachine().getGoalState().getClass().isInstance(state))
                {
                     if (((StateMachine) elevator.getStateMachine()).atState(state)) elevatorCount++;
                }
            }
            
            if(elevatorCount == elevators.size()) elevatorsAt = true;

            //CHECK IF ALL MOTORS ARE AT STATE
             for (StatefulMechanism<?> motor : motors) {
                
                if(motor.getStateMachine().getGoalState().getClass().isInstance(state))
                {
                     if (((StateMachine) motor.getStateMachine()).atState(state)) rollerCount++;
                }
            }
            if(rollerCount == motors.size()) rollersAt = true;

        }
        return (rollersAt && armsAt && elevatorsAt);

    }

    public void requestState(SuperStructureStates states) 
    {   

        currentState = states;
        

        List<Enum<?>> stateList = states.getStates();
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
        // setGoalState(currentState);
        if(!prevStates.equals(currentState))
        {
            System.out.println("Request State");
            requestState(currentState);
        }
        if(nextState != null)
        {
            if(nextState.isValid())
            {
                System.out.println("Moving on");

                requestState(nextState.getTargetState());
                nextState = null;
            }
        }
        arms.forEach(StatefulArmMechanism::update);
        elevators.forEach(StatefulElevatorMechanism::update);
        motors.forEach(StatefulMechanism::update);
        prevStates = currentState;
    }
}
