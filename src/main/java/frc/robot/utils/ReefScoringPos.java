package frc.robot.utils;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ReefScoringPos {

    private final ReefLevel level;
    private final ReefPole side;

     public enum ReefPole {
        
        A(18, 7, new Translation2d(14.3,4), new Translation2d(3.2, 4), "SideA"),
        B(18, 7, new Translation2d(14.3,4), new Translation2d(3.2,4), "SideA"),

        C(17, 8, new Translation2d(13.7,5.1), new Translation2d(3.8,3), "SideC"),
        D(17, 8, new Translation2d(13.7,5.1), new Translation2d(3.8,3), "SideC"),

        E(22, 9, new Translation2d(12.4,5.1), new Translation2d(5.1,2.9), "SideE"),
        F(22, 9, new Translation2d(12.4,5.1), new Translation2d(5.1,2.9), "SideE"),

        G(21, 10, new Translation2d(11.8,4), new Translation2d(5.8,4), "SideG"),
        H(21, 10, new Translation2d(11.8,4), new Translation2d(5.8,4), "SideG"),

        I(20, 11, new Translation2d(12.4,3), new Translation2d(5.1,5.1), "SideI"),
        J(20, 11, new Translation2d(12.4,3), new Translation2d(5.1,5.1), "SideI"),
        
        K(19, 6, new Translation2d(13.7,3), new Translation2d(3.8,5), "SideK"),
        L(19, 6, new Translation2d(13.7,3), new Translation2d(3.8,5), "SideK");
    
        private final long apriltagIdRed, apriltagIdBlue;
        private final Translation2d redPose, bluePose;
        private final String path;
    
        ReefPole(long apriltagIdBlue, long apriltagIdRed, Translation2d redPose, Translation2d bluePose, String path) {
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

        public static ReefPole getPoleFromID(int id)
        {
            switch (id) {
                case 6:
                    return ReefPole.K;  
                case 19:
                    return ReefPole.K;
                case 7:
                    return ReefPole.A;  
                case 18:
                    return ReefPole.A;
                case 8:
                    return ReefPole.C;  
                case 17:
                    return ReefPole.C;
                case 9:
                    return ReefPole.E;  
                case 22:
                    return ReefPole.E;
                case 10:
                    return ReefPole.G;  
                case 21:
                    return ReefPole.G;
                case 11:
                    return ReefPole.I;  
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
            try {
                return PathPlannerPath.fromPathFile(path);
            } catch (FileVersionException | IOException | ParseException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
                return null;
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

    public Translation2d getTranslation() {
        return side.getTranslation();
    }

    public Translation2d getTranslation(Alliance team) {
        return side.getTranslation(team);
    }
}
