// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Experimental;

import java.util.function.BooleanSupplier;

public class Constraint<E extends Enum<E>> {
    private final E targetState;
    private final BooleanSupplier condition;

    public Constraint(E targetState, BooleanSupplier condition) {
        this.targetState = targetState;
        this.condition = condition;
    }

    public boolean isValid(E transition) {
        return !transition.equals(targetState) || condition.getAsBoolean();
    }

    public E getTargetState() {
        return targetState;
    }
}
