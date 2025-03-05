package frc.robot.utils;

import com.pathplanner.lib.path.PathPlannerPath;

import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimeLightConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.PATHS;

public class ReefScoringPos {

    public static LimeLight currentLL = new LimeLight(new LimeLightConfig("empty"));

    public static LimeLight getLimelightFacing(RobotBase<?> base)
    {
      LimeLight ll = base.getVision().getLimelight("limelight-left");
      LimeLight lr = base.getVision().getLimelight("limelight-right");
  
      ReefPole leftPole = ReefPole.getPoleFromID(ll.getAprilTagID(), ll);
      ReefPole rightPole = ReefPole.getPoleFromID(ll.getAprilTagID(), ll);
      if(leftPole != null)
      {
        currentLL = ll;
      }
      if(rightPole != null)
      {
        currentLL = lr;
      }
  
      return currentLL;
    }

    private final ReefLevel level;
    private final ReefPole side;

     public enum ReefPole {
        
        A(18, 7, new Pose2d(11.757,4.200, Rotation2d.fromDegrees(180)), new Pose2d(3.241, 4.200,  Rotation2d.fromDegrees(0)), PATHS.SIDEA),
        B(18, 7, new Pose2d(11.776,3.880, Rotation2d.fromDegrees(180)), new Pose2d(3.211,3.880, Rotation2d.fromDegrees(0)), PATHS.SIDEA),

        C(17, 8, new Pose2d(12.275,3.033,  Rotation2d.fromDegrees(-120)), new Pose2d(3.699,3.033, Rotation2d.fromDegrees(60)), PATHS.SIDEC),
        D(17, 8, new Pose2d(12.574,2.863,  Rotation2d.fromDegrees(-120)), new Pose2d(3.969,2.863, Rotation2d.fromDegrees(60)), PATHS.SIDEC),

        E(22, 9, new Pose2d(13.561,2.863,  Rotation2d.fromDegrees(-60)), new Pose2d(4.956,2.853,  Rotation2d.fromDegrees(120)), PATHS.SIDEE),
        F(22, 9, new Pose2d(13.861,2.993,  Rotation2d.fromDegrees(-60)), new Pose2d(5.245,2.993,  Rotation2d.fromDegrees(120)), PATHS.SIDEE),

        G(21, 10, new Pose2d(14.369,3.860, Rotation2d.fromDegrees(0)), new Pose2d(5.754,3.860, Rotation2d.fromDegrees(180)), PATHS.SIDEG),
        H(21, 10, new Pose2d(14.359,4.200, Rotation2d.fromDegrees(0)), new Pose2d(5.754,4.200, Rotation2d.fromDegrees(180)), PATHS.SIDEG),

        I(20, 11, new Pose2d(13.880,5.007, Rotation2d.fromDegrees(60)), new Pose2d(5.285,5.007,  Rotation2d.fromDegrees(-120)), PATHS.SIDEI),
        J(20, 11, new Pose2d(13.561,5.207, Rotation2d.fromDegrees(60)), new Pose2d(4.986,5.207,  Rotation2d.fromDegrees(-120)), PATHS.SIDEI),
        
        K(19, 6, new Pose2d(12.594,5.187, Rotation2d.fromDegrees(120)), new Pose2d(3.989,5.187, Rotation2d.fromDegrees(-60)), PATHS.SIDEK),
        L(19, 6, new Pose2d(12.325,5.027, Rotation2d.fromDegrees(120)), new Pose2d(3.719,5.027, Rotation2d.fromDegrees(-60)), PATHS.SIDEK);
    
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

        public static Translation2d getCenterReef()
        {
            if(DriverStation.getAlliance().orElse(Robot.isSimulation() ? Alliance.Blue : null).equals(Alliance.Blue))
            {
                return new Translation2d(4.477, 4);
            }
            else
            {
                return new Translation2d(13.063, 4);
            }
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

        public static ReefPole getPoleFromID(long id, LimeLight ll)
        {
            switch ((int)id) {
                case 6:
                case 19:
                    if(ll.config.table() == "limelight-left")
                    {
                        return ReefPole.L;
                    }
                    else
                    {
                        return ReefPole.K;
                    }
                case 7:
                case 18:
                if(ll.config.table() == "limelight-left")
                {
                    return ReefPole.B;
                }
                else
                {
                    return ReefPole.A;
                }
                case 8:
                case 17:
                if(ll.config.table() == "limelight-left")
                {
                    return ReefPole.D;
                }
                else
                {
                    return ReefPole.C;
                }
                case 9:
                case 22:
                if(ll.config.table() == "limelight-left")
                {
                    return ReefPole.F;
                }
                else
                {
                    return ReefPole.E;
                }
                case 10:
                case 21:
                if(ll.config.table() == "limelight-left")
                {
                    return ReefPole.H;
                }
                else
                {
                    return ReefPole.G;
                }
                case 11:
                case 20:
                if(ll.config.table() == "limelight-left")
                {
                    return ReefPole.J;
                }
                else
                {
                    return ReefPole.I;
                }

                default:
                    return null;
            }
        }

        public Translation2d getTranslation() {
            return getTranslation(DriverStation.getAlliance().orElse(Robot.isSimulation() ? Alliance.Blue : null));
         }
     
         public Translation2d getTranslation(Alliance team) {
            return getPose2d(team).getTranslation();
         }

         public Rotation2d getRotation() {
            return getRotation(DriverStation.getAlliance().orElse(Robot.isSimulation() ? Alliance.Blue : null));
         }
     
         public Rotation2d getRotation(Alliance team) {
            return getPose2d(team).getRotation();
         }

         public Pose2d getPose2d() {
            return getPose2d(DriverStation.getAlliance().orElse(Robot.isSimulation() ? Alliance.Blue : null));
         }
     
         public Pose2d getPose2d(Alliance team) {
             switch (team) {
                 case Red:
                     return redPose;
                 case Blue:
                     return bluePose;
                 default:
                     DriverStation.reportError("Cant Find Team!!!", new IllegalArgumentException("Unknown team: " + team).getStackTrace());
                     return null;
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
