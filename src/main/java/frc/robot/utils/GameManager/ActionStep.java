package frc.robot.utils.GameManager;

public interface ActionStep {
    /** Called once when the step starts */
    void start();

    /** Called periodically until finished */
    void update();

    /** When true, sequence advances */
    boolean isFinished();

    /** Called once when step exits */
    default void end(boolean interrupted) {}
}
