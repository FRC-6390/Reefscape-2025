package frc.robot.utils;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer.PATHS;

public class ReefScoringPos {

    private final ReefLevel level;
    private final ReefPole side;

     public enum ReefPole {
        
        A(18, 7, new Pose2d(14.3,4, Rotation2d.fromDegrees(180)), new Pose2d(3.2, 4,  Rotation2d.fromDegrees(0)), PATHS.SIDEA),
        B(18, 7, new Pose2d(14.3,4, Rotation2d.fromDegrees(180)), new Pose2d(3.2,4, Rotation2d.fromDegrees(0)), PATHS.SIDEA),

        C(17, 8, new Pose2d(13.7,5.1,  Rotation2d.fromDegrees(-120)), new Pose2d(3.8,3, Rotation2d.fromDegrees(60)), PATHS.SIDEC),
        D(17, 8, new Pose2d(13.7,5.1,  Rotation2d.fromDegrees(-120)), new Pose2d(3.8,3, Rotation2d.fromDegrees(60)), PATHS.SIDEC),

        E(22, 9, new Pose2d(12.4,5.1,  Rotation2d.fromDegrees(-60)), new Pose2d(5.1,2.9,  Rotation2d.fromDegrees(120)), PATHS.SIDEE),
        F(22, 9, new Pose2d(12.4,5.1,  Rotation2d.fromDegrees(-60)), new Pose2d(5.1,2.9,  Rotation2d.fromDegrees(120)), PATHS.SIDEE),

        G(21, 10, new Pose2d(11.8,4, Rotation2d.fromDegrees(0)), new Pose2d(5.8,4, Rotation2d.fromDegrees(180)), PATHS.SIDEG),
        H(21, 10, new Pose2d(11.8,4, Rotation2d.fromDegrees(0)), new Pose2d(5.8,4, Rotation2d.fromDegrees(180)), PATHS.SIDEG),

        I(20, 11, new Pose2d(12.4,3, Rotation2d.fromDegrees(60)), new Pose2d(5.1,5.1,  Rotation2d.fromDegrees(-120)), PATHS.SIDEI),
        J(20, 11, new Pose2d(12.4,3, Rotation2d.fromDegrees(60)), new Pose2d(5.1,5.1,  Rotation2d.fromDegrees(-120)), PATHS.SIDEI),
        
        K(19, 6, new Pose2d(13.7,3, Rotation2d.fromDegrees(120)), new Pose2d(3.8,5, Rotation2d.fromDegrees(-60)), PATHS.SIDEK),
        L(19, 6, new Pose2d(13.7,3, Rotation2d.fromDegrees(120)), new Pose2d(3.8,5, Rotation2d.fromDegrees(-60)), PATHS.SIDEK);
    
        private final long apriltagIdRed, apriltagIdBlue;
        private final Pose2d redPose, bluePose;
        private final PATHS path;
    
        ReefPole(long apriltagIdBlue, long apriltagIdRed, Pose2d redPose, Pose2d bluePose, PATHS path) {
            this.apriltagIdRed = apriltagIdRed;
            this.apriltagIdBlue = apriltagIdBlue;
            this.redPose = redPose;
            this.bluePose = bluePose;
            this.path = path;

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

        public static ReefPole getPoleFromID(long id)
        {
            switch ((int)id) {
                case 6:
                case 19:
                    return ReefPole.K;
                case 7:
                case 18:
                    return ReefPole.A;
                case 8:
                case 17:
                    return ReefPole.C;
                case 9:
                case 22:
                    return ReefPole.E;
                case 10:
                case 21:
                    return ReefPole.G;
                case 11:
                case 20:
                    return ReefPole.I;

                default:
                    return null;
            }
        }

        public Translation2d getTranslation() {
            return getTranslation(DriverStation.getAlliance().get());
         }
     
         public Translation2d getTranslation(Alliance team) {
            return getPose2d(team).getTranslation();
         }

         public Rotation2d getRotation() {
            return getRotation(DriverStation.getAlliance().get());
         }
     
         public Rotation2d getRotation(Alliance team) {
            return getPose2d(team).getRotation();
         }

         public Pose2d getPose2d() {
            return getPose2d(DriverStation.getAlliance().get());
         }
     
         public Pose2d getPose2d(Alliance team) {
             switch (team) {
                 case Red:
                     return redPose;
                 case Blue:
                     return bluePose;
                 default:
                     throw new IllegalArgumentException("Unknown team: " + team);
             }
         }

         public PathPlannerPath getPath()
         {
           return path.getPath();
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

    public Translation2d getTranslation() {
        return side.getTranslation();
    }

    public Translation2d getTranslation(Alliance team) {
        return side.getTranslation(team);
    }

    public Rotation2d getRotation() {
        return side.getRotation();
    }

    public Rotation2d getRotation(Alliance team) {
        return side.getRotation(team);
    }

    public Pose2d getPose2d() {
        return side.getPose2d();
    }

    public Pose2d getPose2d(Alliance team) {
        return side.getPose2d(team);
    }
}
