// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.Experimental;

import java.util.function.BooleanSupplier;

public class Constraint<SuperStructureStates extends Enum<SuperStructureStates>> {
    private final SuperStructureStates targetState;
    private final BooleanSupplier condition;

    public Constraint(SuperStructureStates targetState, BooleanSupplier condition) {
        this.targetState = targetState;
        this.condition = condition;
    }

    public boolean isValid() {
        return condition.getAsBoolean();
    }

    public SuperStructureStates getTargetState() {
        return targetState;
    }
}
