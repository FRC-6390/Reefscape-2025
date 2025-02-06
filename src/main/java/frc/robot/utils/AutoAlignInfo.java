package frc.robot.utils;

import javax.print.attribute.standard.Media;

import ca.frc6390.athena.core.RobotLocalization;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.filters.FilterList;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateType;
import ca.frc6390.athena.sensors.camera.limelight.LimelightConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AutoAlignInfo 
{
 public Rotation2d lastYaw = new Rotation2d();
 public Rotation2d lastRobotYaw = new Rotation2d();
 public Pose2d botpose = new Pose2d();
 public double xMeasurement = 0;
 public LimeLight limeLight;
 public RobotLocalization localization;
 public double thetaMeasurement = 0;
 public Pose2d lastRobotPose2d = new Pose2d();
 public SwerveDrivetrain drivetrain;
 public MedianFilter filter = new MedianFilter(10);
 public FilterList x = new FilterList().addMedianFilter(10);
 public FilterList y = new FilterList().addMedianFilter(50);
 


 public AutoAlignInfo(LimeLight limeLight, RobotLocalization localization, SwerveDrivetrain drivetrain)
 {
    this.limeLight = limeLight;
    this.localization = localization;
 }

 public void gatherData()
 {
    thetaMeasurement = -filter.calculate(limeLight.getPoseEstimate(PoseEstimateType.TARGET_POSE_ROBOT_SPACE).getRaw()[4]);
    lastYaw = Rotation2d.fromDegrees(thetaMeasurement);
    lastRobotYaw = Rotation2d.fromRadians(MathUtil.angleModulus(drivetrain.getIMU().getYaw().getRadians()));
    lastRobotPose2d = localization.getRelativePose();
    localization.resetRelativePose(0,  0, 0);
    Pose2d pos = limeLight.getPoseEstimate(PoseEstimateType.TARGET_POSE_ROBOT_SPACE).getPose();
    botpose = new Pose2d(-x.calculate(pos.getX()), -y.calculate(pos.getY()), pos.getRotation());
    xMeasurement = limeLight.getTargetHorizontalOffset();
}
public double getThetatMeasurement()
{
    return thetaMeasurement;
}
public double getXMeasurement()
{
    return xMeasurement;
}
public Rotation2d getLastYaw()
{
    return lastYaw;
}
public Rotation2d getLastRobotYaw()
{
    return lastRobotYaw;
}
public Pose2d getLastRobotPose()
{
    return lastRobotPose2d;
}
public Pose2d getTargetPoseRobotSpace()
{
    return botpose;
}
}
