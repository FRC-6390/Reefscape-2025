package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ReefScoringPos {

    private final ReefLevel level;
    private final ReefPole side;

     public enum ReefPole {
        
        A(18, 7, new Translation2d(1,1), new Translation2d(1,1)),
        B(18, 7, new Translation2d(1,1), new Translation2d(1,1)),
        C(17, 8, new Translation2d(1,1), new Translation2d(1,1)),
        D(17, 8, new Translation2d(1,1), new Translation2d(1,1)),
        E(22, 9, new Translation2d(1,1), new Translation2d(1,1)),
        F(22, 9, new Translation2d(1,1), new Translation2d(1,1)),
        G(21, 10, new Translation2d(1,1), new Translation2d(1,1)),
        H(21, 10, new Translation2d(1,1), new Translation2d(1,1)),
        I(20, 11, new Translation2d(1,1), new Translation2d(1,1)),
        J(20, 11, new Translation2d(1,1), new Translation2d(1,1)),
        K(19, 6, new Translation2d(1,1), new Translation2d(1,1)),
        L(19, 6, new Translation2d(1,1), new Translation2d(1,1));
    
        private final long apriltagIdRed, apriltagIdBlue;
        private final Translation2d redPose, bluePose;
    
        ReefPole(long apriltagIdBlue, long apriltagIdRed, Translation2d redPose, Translation2d bluePose) {
            this.apriltagIdRed = apriltagIdRed;
            this.apriltagIdBlue = apriltagIdBlue;
            this.redPose = redPose;
            this.bluePose = bluePose;
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
                    throw new IllegalArgumentException("Unknown team: " + team);
            }
        }

        public Translation2d getPose() {
            return getPose(DriverStation.getAlliance().get());
         }
     
         public Translation2d getPose(Alliance team) {
             switch (team) {
                 case Red:
                     return redPose;
                 case Blue:
                     return bluePose;
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

    public long getApriltagId() {
        return side.getApriltagId();
    }

    public long getApriltagId(Alliance team) {
        return side.getApriltagId(team);
    }

    public Translation2d getPose() {
        return side.getPose();
    }

    public Translation2d getPose(Alliance team) {
        return side.getPose(team);
    }
}
