package frc.robot.utils.Align;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
public class AprilTagMap {
    
     public enum AprilTags {

        NONE(new AprilTag(-1, -1, new Pose2d(), new Pose2d(), Rotation2d.fromDegrees(0))),

        CLIMB(new AprilTag(15, 31, new Pose2d(), new Pose2d(), Rotation2d.fromDegrees(0))),
        
        TOP(new AprilTag(0, 0, new Pose2d(), new Pose2d(), Rotation2d.fromDegrees(0))),


        HUB(new AprilTag(120, 02, new Pose2d(), new Pose2d(), Rotation2d.fromDegrees(0)));

        public AprilTag tag;
        AprilTags(AprilTag tag)
        {
            this.tag = tag;
        }

        public AprilTag getAprilTag()
        {
            return tag;
        }
}
}