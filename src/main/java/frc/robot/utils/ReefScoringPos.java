package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ReefScoringPos {

    private final ReefLevel level;
    private final ReefPole side;

     public enum ReefPole {

        NONE(-1, -1, new Pose2d()),

        //LEFT
        A(18, 7,new Pose2d(0, 0.0,new Rotation2d())),
        //RIGHT
        B(18, 7, new Pose2d(0, 0.0,new Rotation2d())),

        C(17, 8,new Pose2d(0, 0.0,new Rotation2d())),
        D(17, 8, new Pose2d(0, 0.0,new Rotation2d())),

        E(22, 9,  new Pose2d(0, 0.0,new Rotation2d())),
        F(22, 9,  new Pose2d(0, 0.0,new Rotation2d())),

        G(21, 10,  new Pose2d(0, 0.0,new Rotation2d())),
        H(21, 10,  new Pose2d(0, 0.0,new Rotation2d())),

        I(20, 11,  new Pose2d(0.37, 0.34,new Rotation2d())),
        J(20, 11, new Pose2d(0.12, 0.47,new Rotation2d())),

        K(19, 6, new Pose2d(-0.1, 0.48,new Rotation2d())),
        L(19, 6,  new Pose2d(-0.1, 0.48,new Rotation2d()));

        private final long apriltagIdRed, apriltagIdBlue;
        private final Pose2d offset;
    
        ReefPole(long apriltagIdBlue, long apriltagIdRed,Pose2d offset) {
            this.apriltagIdRed = apriltagIdRed;
            this.apriltagIdBlue = apriltagIdBlue;
            this.offset = offset;

        }

        public Pose2d getOffset() {
            return offset;
        }

        public long getApriltagId() {
           return getApriltagId(DriverStation.getAlliance().get());
        }
    
        public long getApriltagId(Alliance team) {
            switch (team) {
                case Red:
                    return apriltagIdRed;
                case Blue:
                    return apriltagIdBlue;
                default:
                    DriverStation.reportError("Cant Find Team!!!", new IllegalArgumentException("Unknown team: " + team).getStackTrace());
                    return -1;
            }
        }

        public static ReefPole getPoleFromID(long id, boolean isRightPole)
        {
            switch ((int) id) {
                case 6:
                case 19:
                    return isRightPole ?  ReefPole.K : ReefPole.L;
                case 7:
                case 18:
                    return isRightPole ?  ReefPole.A : ReefPole.B;
                case 8:
                case 17:
                    return isRightPole ?  ReefPole.C : ReefPole.D;
                case 9:
                case 22:
                    return isRightPole ?  ReefPole.E : ReefPole.F;
                case 10:
                case 21:
                    return isRightPole ?  ReefPole.G : ReefPole.H;
                case 11:
                case 20:
                    return isRightPole ?  ReefPole.I : ReefPole.J;
                default:
                    return ReefPole.NONE;
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

    public long getApriltagId() {
        return side.getApriltagId();
    }

    public long getApriltagId(Alliance team) {
        return side.getApriltagId(team);
    }
}
