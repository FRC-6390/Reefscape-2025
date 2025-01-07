package frc.robot.utils;

public class ReefScoringPos {

    /**
     * Read the positions relative to the current
     * driver station clockwise
     *           1
     *         -----
     *     6  /     \ 2
     *     5  \     / 3
     *         -----
     *           4 
     * ------------------------
     * |  DS1 |  DS2  |  DS3  |
     * ------------------------
     *       L     R
     *       _     _
     *      | |   | |
     *      | |   | |
     *      | |   | |
     *      | |   | |
     *      | |   | |
     * |-----------------|
     * |      ___        |
     * |     |   |       |
     * |      ‾‾‾        |
     * 
     * Poles are read relative to the april tag (or if you were facing the side)
     */

    public enum ReefSide {
        ONE_LEFT,
        ONE_RIGHT,
        TWO_LEFT,
        TWO_RIGHT,
        THREE_LEFT,
        THREE_RIGHT,
        FOUR_LEFT,
        FOUR_RIGHT,
        FIVE_LEFT,
        FIVE_RIGHT,
        SIX_LEFT,
        SIX_RIGHT
    }

    public enum ReefLevel {
        L1,
        L2,
        L3,
        L4
    }

    public ReefScoringPos(ReefLevel level, ReefSide side) {

    }
}
