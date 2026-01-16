package frc.robot.utils.GameManager;

import ca.frc6390.athena.core.RobotCore;
import ca.frc6390.athena.mechanisms.SuperstructureMechanism;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ObjectivePlanner {

    private final SuperstructureMechanism superstructure;
    private final RobotCore<?> base;

    public ObjectivePlanner(
        SuperstructureMechanism superstructure,
        RobotCore<?> base
    ) {
        this.superstructure = superstructure;
        this.base = base;
    }

    public Command executeObjective(Objective objective) {
        return Commands.sequence(
            objective.position.driveTo(null, null, null),
            new ObjectiveCommand(objective, superstructure)
        );
    }
}
