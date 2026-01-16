package frc.robot.utils.GameManager;

import frc.robot.utils.GameManager.ActionSequence;

import frc.robot.Robot;
import frc.robot.Constants.Superstructure.SuperstructureState;

public enum ActionSequences {

    CLIMB {
        @Override
        public ActionSequence build(Robot robot) {
            return new ActionSequence(
                new TimedActionStep(
                    () -> robot.setSuper(SuperstructureState.L1),
                    () -> robot.superstructure.childrenAtGoals(),
                    1.5
                ),
                new TimedActionStep(
                    () -> robot.setSuper(SuperstructureState.Home),
                    () -> robot.superstructure.childrenAtGoals(),
                    1.5
                )
            );
        }
    },

    SCORE_L4 {
        @Override
        public ActionSequence build(Robot robot) {
            return new ActionSequence(
                new TimedActionStep(
                    () -> robot.setSuper(SuperstructureState.L4),
                    () -> robot.superstructure.childrenAtGoals(),
                    1.5
                ),
                new TimedActionStep(
                    () -> robot.setSuper(SuperstructureState.Score),
                    () -> true,
                    0.5
                )
            );
        }
    };

    public abstract ActionSequence build(Robot robot);
}
