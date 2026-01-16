package frc.robot.utils.GameManager;

import frc.robot.utils.Align.RelevantPosition;
import java.util.function.BooleanSupplier;

public class Objective {

    public final RelevantPosition position;
    public final ActionSequence sequence;

    public final BooleanSupplier isComplete;
    public final BooleanSupplier isFailed;

    public Objective(
        RelevantPosition position,
        ActionSequence sequence,
        BooleanSupplier isComplete,
        BooleanSupplier isFailed
    ) {
        this.position = position;
        this.sequence = sequence;
        this.isComplete = isComplete;
        this.isFailed = isFailed;
    }
}
