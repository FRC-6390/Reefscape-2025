package frc.robot.utils.GameManager;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ObjectiveCommand extends Command {

    private final Objective objective;
    private final Subsystem[] requirements;

    public ObjectiveCommand(Objective objective, Subsystem... requirements) {
        this.objective = objective;
        this.requirements = requirements;
        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        objective.sequence.reset();
    }

    @Override
    public void execute() {
        objective.sequence.execute();
    }

    @Override
    public void end(boolean interrupted) {
        objective.sequence.interrupt();
    }

    @Override
    public boolean isFinished() {
        return objective.isComplete.getAsBoolean()
            || objective.isFailed.getAsBoolean()
            || objective.sequence.isFinished();
    }
}
