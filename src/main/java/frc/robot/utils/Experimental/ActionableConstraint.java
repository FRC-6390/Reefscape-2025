// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.Experimental;

import java.util.function.BooleanSupplier;

public class ActionableConstraint<SuperStructureStates extends Enum<SuperStructureStates>> {
    private final SuperStructureStates targetState;
    private final SuperStructureStates transitionState;
    private final BooleanSupplier condition;

    public ActionableConstraint(SuperStructureStates targetState, SuperStructureStates transitionState, BooleanSupplier condition) {
        this.targetState = targetState;
        this.condition = condition;
        this.transitionState = transitionState;
    }

    public boolean isValid() {
        return condition.getAsBoolean();
    }

    public SuperStructureStates getTargetState() {
        return targetState;
    }
     public SuperStructureStates getTransition() {
        return transitionState;
    }
}
