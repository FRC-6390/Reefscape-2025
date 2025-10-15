package frc.robot.subsystems.Experimental;

import java.util.*;

import ca.frc6390.athena.mechanisms.StatefulMechanism;
import ca.frc6390.athena.mechanisms.ArmMechanism.StatefulArmMechanism;
import ca.frc6390.athena.mechanisms.ElevatorMechanism.StatefulElevatorMechanism;
import ca.frc6390.athena.mechanisms.SimpleMotorMechanism.StatefulSimpleMotorMechanism;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;

public class SuperstructureBuilder {

    private final List<StatefulArmMechanism<?>> arms = new ArrayList<>();
    private final List<StatefulElevatorMechanism<?>> elevators = new ArrayList<>();
    private final List<StatefulMechanism<?>> motors = new ArrayList<>();
    private final List<Constraint<?>> constraints = new ArrayList<>();
    private final List<ActionableConstraint<?>> actionableConstraints = new ArrayList<>();


    private SuperstructureBuilder() {
    }

    public static SuperstructureBuilder builder() {
        return new SuperstructureBuilder();
    }

    public SuperstructureBuilder addArms(StatefulArmMechanism<?>... mechs) {
        arms.addAll(Arrays.asList(mechs));
        return this;
    }

    public SuperstructureBuilder addElevators(StatefulElevatorMechanism<?>... mechs) {
        elevators.addAll(Arrays.asList(mechs));
        return this;
    }

    public SuperstructureBuilder addMotors(StatefulMechanism<?>... mechs) {
        motors.addAll(Arrays.asList(mechs));
        return this;
    }

    public SuperstructureBuilder addConstraint(Constraint<?> constraint) {
        constraints.add(constraint);
        return this;
    }

    public SuperstructureBuilder addActionableConstraint(ActionableConstraint<?> constraint) {
        actionableConstraints.add(constraint);
        return this;
    }


    public SuperStructureTest build() {
        // Replace nulls with empty lists if necessary
        return new SuperStructureTest(
                arms,
                elevators.isEmpty() ? Collections.emptyList() : elevators,
                motors.isEmpty() ? Collections.emptyList() : motors,
                constraints.isEmpty() ? Collections.emptyList() : constraints,
                actionableConstraints.isEmpty() ? Collections.emptyList() : actionableConstraints

        );
    }
}
