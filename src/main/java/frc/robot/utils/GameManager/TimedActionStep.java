package frc.robot.utils.GameManager;

import edu.wpi.first.wpilibj.Timer;
import java.util.function.BooleanSupplier;

public class TimedActionStep implements ActionStep {

    private final Runnable onStart;
    private final BooleanSupplier finishedCondition;
    private final double timeoutSeconds;

    private final Timer timer = new Timer();

    public TimedActionStep(
        Runnable onStart,
        BooleanSupplier finishedCondition,
        double timeoutSeconds
    ) {
        this.onStart = onStart;
        this.finishedCondition = finishedCondition;
        this.timeoutSeconds = timeoutSeconds;
    }

    @Override
    public void start() {
        timer.reset();
        timer.start();
        onStart.run();
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return finishedCondition.getAsBoolean() || timer.hasElapsed(timeoutSeconds);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }
}
