package frc.robot.commands.GoingBald;

import com.pathplanner.lib.path.PathPlannerPath;

import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimeLightConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Autos;
import frc.robot.Robot;
public class AprilTagMap {

    

     public enum AprilTags {

        NONE(-1, -1, Rotation2d.fromDegrees(0)),

        A(-1, -1, Rotation2d.fromDegrees(0)),

        C(-1, -1, Rotation2d.fromDegrees(0)),

        E(-1, -1, Rotation2d.fromDegrees(0)),
        
        G(-1, -1, Rotation2d.fromDegrees(0)),

        I(-1, -1, Rotation2d.fromDegrees(0)),

        K(-1, -1, Rotation2d.fromDegrees(0));

        private final long redId;
        private final long blueId;


        private final Rotation2d rotation;
    
        AprilTags(long redid,long blueid,Rotation2d rotation) {
            this.rotation = rotation;
            this.redId = redid;
            this.blueId = blueid;
        }

        public long getApriltagId() {
           return getApriltagId(DriverStation.getAlliance().get());
        }

        
        public long getApriltagId(Alliance team) {
            switch (team) {
                case Red:
                    return redId;
                case Blue:
                    return blueId;
                default:
                    DriverStation.reportError("Cant Find Team!!!", new IllegalArgumentException("Unknown team: " + team).getStackTrace());
                    return -1;
            }
        }

        public static Rotation2d getRotation2d(long id)
        {
            Rotation2d rot = new Rotation2d();
            for (AprilTags tag : AprilTags.values()) {
                if(tag.getApriltagId() == id)
                {
                    rot = tag.rotation;
                }   
            } 
            return rot;
        }
}
}