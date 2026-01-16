package frc.robot.utils.GameManager;

import java.util.List;

public class ActionSequence {

    private final List<ActionStep> steps;
    private int index = 0;
    private ActionStep currentStep = null;

    public ActionSequence(ActionStep... steps) {
        this.steps = List.of(steps);
    }

    public void reset() {
        index = 0;
        currentStep = null;
    }

    public void execute() {
        if (isFinished()) return;

        if (currentStep == null) {
            currentStep = steps.get(index);
            currentStep.start();
        }

        currentStep.update();

        if (currentStep.isFinished()) {
            currentStep.end(false);
            index++;
            currentStep = null;
        }
    }

    public void interrupt() {
        if (currentStep != null) {
            currentStep.end(true);
        }
    }

    public boolean isFinished() {
        return index >= steps.size();
    }
}
