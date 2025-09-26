package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.ReefScoringPos.ReefPole;
import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateType;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatencyType;

public class V3 {
    

    class V3Helper {

        // public void DEBUG_POLE_POSE(RobotBase<?> base, LimeLight camera) {
        //     Pose2d pole = ReefPole.getPoleFromID(camera.getAprilTagID(), camera).getPose2d();
        //     base.getLocalization().getField2dObject("flipped").setPose(new Pose2d(pose.getY()+pole.getX(), pose.getX()+pole.getY(), pose.getRotation().plus(pole.getRotation())));    
        //     base.getLocalization().getField2dObject("RobotPose").setPose(pose.relativeTo(ReefPole.getPoleFromID(camera_right.getAprilTagID(), camera).getPose2d()));
        //     base.getLocalization().getField2dObject("Tag").setPose(ReefPole.getPoleFromID(camera.getAprilTagID(), camera).getPose2d());
        // }

        public Translation2d targetAverageTargetTranslation(RobotBase<?> base, LimeLight... limeLights) {
            double x = 0;
            double y = 0;
            double count = 0;
            for (LimeLight limeLight : limeLights) {
                if (limeLight.hasValidTarget()) {
                    count++;
                    double dist1 = limeLight.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9];
                    double angle1 = limeLight.getTargetHorizontalOffset() -base.getLocalization().getRelativePose().getRotation().getDegrees();
                    x += (Math.cos(Math.toRadians(angle1)) * dist1) - Units.inchesToMeters(5);
                    y += (Math.sin(Math.toRadians(angle1)) * dist1) - Units.inchesToMeters(10);
                }
            }
            return new Translation2d(x/count, y/count);
        }
        
        public boolean hasValidTarget(LimeLight... limeLights) {
            for (LimeLight limeLight : limeLights) {
                if (limeLight.hasValidTarget()) {
                    return true;
                }
            }
            return false;
        }

        public ChassisSpeeds createRelativeSpeeds(RobotBase<?> base, double xSpeed, double ySpeed, double rot) {
            return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(xSpeed, ySpeed, rot), base.getLocalization().getRelativePose().getRotation());
        }
    }
}
