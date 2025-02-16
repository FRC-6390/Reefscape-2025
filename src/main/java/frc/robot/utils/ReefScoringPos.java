package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ReefScoringPos {

    private final ReefLevel level;
    private final ReefPole side;

     public enum ReefPole {
        
        A(18, 7),
        B(18, 7),
        C(17, 8),
        D(17, 8),
        E(22, 9),
        F(22, 9),
        G(21, 10),
        H(21, 10),
        I(20, 11),
        J(20, 11),
        K(19, 6),
        L(19, 6);
    
        private final long apriltagIdRed;
        private final long apriltagIdBlue;
    
        ReefPole(long apriltagIdBlue, long apriltagIdRed) {
            this.apriltagIdRed = apriltagIdRed;
            this.apriltagIdBlue = apriltagIdBlue;
        }
    
        public long getApriltagId(Alliance team) {
            switch (team) {
                case Red:
                    return apriltagIdRed;
                case Blue:
                    return apriltagIdBlue;
                default:
                    throw new IllegalArgumentException("Unknown team: " + team);
            }
        }
    }

    public enum ReefLevel {
        L1,
        L2,
        L3,
        L4
    }

    public ReefScoringPos(ReefLevel level, ReefPole side) {
        this.level = level;
        this.side = side;
    }

    public ReefLevel getLevel() {
        return level;
    }

    public ReefPole getSide() {
        return side;
    }

    public long getApriltagId(Alliance team) {
        return side.getApriltagId(team);
    }

}
