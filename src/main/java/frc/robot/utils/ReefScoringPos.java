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
import frc.robot.Autos;
import frc.robot.Robot;
public class ReefScoringPos {

    public static LimeLight currentLL = new LimeLight(new LimeLightConfig("limelight-left"));

    public static LimeLight getLimelightFacing(RobotBase<?> base)
    {
      LimeLight ll = base.getVision().getLimelight("limelight-left");
      LimeLight lr = base.getVision().getLimelight("limelight-right");
  
      ReefPole leftPole = ReefPole.getPoleFromID(ll.getAprilTagID(), ll);
      ReefPole rightPole = ReefPole.getPoleFromID(lr.getAprilTagID(), lr);
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

        NONE(-1, -1, new Pose2d(), new Pose2d(), null, 0, new Pose2d()),

        
        A(18, 7, new Pose2d(11.757,4.200, Rotation2d.fromDegrees(0)), new Pose2d(3.241, 4.200,  Rotation2d.fromDegrees(180)), Autos.PATHS.SIDEA,0.0, new Pose2d()),
        B(18, 7, new Pose2d(11.776,3.880, Rotation2d.fromDegrees(0)), new Pose2d(3.211,3.880, Rotation2d.fromDegrees(180)), Autos.PATHS.SIDEA,0.0, new Pose2d()),

        C(17, 8, new Pose2d(12.275,3.033,  Rotation2d.fromDegrees(60)), new Pose2d(3.699,3.033, Rotation2d.fromDegrees(-120)), Autos.PATHS.SIDEC,0.0, new Pose2d()),
        D(17, 8, new Pose2d(12.574,2.863,  Rotation2d.fromDegrees(60)), new Pose2d(3.969,2.863, Rotation2d.fromDegrees(-120)), Autos.PATHS.SIDEC,0.0, new Pose2d()),

        E(22, 9, new Pose2d(13.561,2.863,  Rotation2d.fromDegrees(120)), new Pose2d(4.956,2.853,  Rotation2d.fromDegrees(-60)), Autos.PATHS.SIDEE,0.0, new Pose2d()),
        F(22, 9, new Pose2d(13.861,2.993,  Rotation2d.fromDegrees(120)), new Pose2d(5.245,2.993,  Rotation2d.fromDegrees(-60)), Autos.PATHS.SIDEE,0.0, new Pose2d()),

        G(21, 10, new Pose2d(14.369,3.860, Rotation2d.fromDegrees(180)), new Pose2d(5.754,3.860, Rotation2d.fromDegrees(0)), Autos.PATHS.SIDEG,0.0, new Pose2d()),
        H(21, 10, new Pose2d(14.359,4.200, Rotation2d.fromDegrees(180)), new Pose2d(5.754,4.200, Rotation2d.fromDegrees(0)), Autos.PATHS.SIDEG,0.0, new Pose2d()),

        I(20, 11, new Pose2d(13.880,5.007, Rotation2d.fromDegrees(-120)), new Pose2d(5.285,5.007,  Rotation2d.fromDegrees(60)), Autos.PATHS.SIDEI,0.0, new Pose2d(0.3464769753795864,0.13341659643371773,Rotation2d.fromDegrees(-120.65686035156251))),
        J(20, 11, new Pose2d(13.561,5.207, Rotation2d.fromDegrees(-120)), new Pose2d(4.986,5.207,  Rotation2d.fromDegrees(60)), Autos.PATHS.SIDEI,0.0, new Pose2d(0.3696211423562274, 0.643746976507688, Rotation2d.fromDegrees(-120.65686035156251))),
        
        K(19, 6, new Pose2d(12.594,5.187, Rotation2d.fromDegrees(-60)), new Pose2d(3.989,5.187, Rotation2d.fromDegrees(120)), Autos.PATHS.SIDEK,0.0, new Pose2d(-0.11716678946362938, 0.11170796544202566, Rotation2d.fromDegrees(-60.8))),
        L(19, 6, new Pose2d(12.325,5.027, Rotation2d.fromDegrees(-60)), new Pose2d(3.719,5.027, Rotation2d.fromDegrees(120)), Autos.PATHS.SIDEK,0.0, new Pose2d(-0.09835251441621518, 0.6477634322326651, Rotation2d.fromDegrees(-60.8)));
    
        private final long apriltagIdRed, apriltagIdBlue;
        private final Pose2d redPose, bluePose;
        private final Autos.PATHS path;
        private final double offsetInDegrees;
        private final Pose2d relativeScoringPos;
    
        ReefPole(long apriltagIdBlue, long apriltagIdRed, Pose2d redPose, Pose2d bluePose, Autos.PATHS path, double offsetInMetres,Pose2d relativeScoringPos) {
            this.apriltagIdRed = apriltagIdRed;
            this.apriltagIdBlue = apriltagIdBlue;
            this.redPose = redPose;
            this.bluePose = bluePose;
            this.path = path;
            this.offsetInDegrees = offsetInMetres;
            this.relativeScoringPos = relativeScoringPos;

        }

        public long getApriltagId() {
           return getApriltagId(DriverStation.getAlliance().get());
        }

        public double getOffsetInDegrees() {
            return offsetInDegrees;
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

        public Pose2d getScoringPos()
        {
            return relativeScoringPos;
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
                    return ReefPole.NONE;
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
