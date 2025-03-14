package frc.robot.utils;

import ca.frc6390.athena.core.RobotDrivetrain;
import ca.frc6390.athena.core.RobotLocalization;
import ca.frc6390.athena.filters.FilterList;
import ca.frc6390.athena.filters.FilteredValue;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoAlignHelper 
{
 public Rotation2d lastYaw = new Rotation2d();
 public Rotation2d lastRobotYaw = new Rotation2d();
 public Pose2d botpose = new Pose2d();
 public double xMeasurement = 0;
 public LimeLight limeLight;
 public RobotLocalization localization;
 public ChassisSpeeds speeds;
 public double thetaMeasurement = 0;
 public Pose2d lastRobotPose2d = new Pose2d();
 public RobotDrivetrain<?> drivetrain;
 public MedianFilter filter = new MedianFilter(10);
 public FilterList x = new FilterList().addMedianFilter(10);
 public FilterList y = new FilterList().addMedianFilter(50);
 public double xVelocity;
 public double yVelocity;
 public double rotationalVelocity;
 public PIDController controller = new PIDController(0.025, 0, 0);
 public ProfiledPIDController xController = new ProfiledPIDController(0.05, 0, 0.0001, new Constraints(4, 2));
 public PIDController xController2 = new PIDController(1.2, 0,0);
 public FilterList xError = new FilterList().addMedianFilter(30);

 public AutoAlignHelper(LimeLight limeLight, RobotLocalization localization, RobotDrivetrain<?> drivetrain)
 {
    this.limeLight = limeLight;
    this.drivetrain = drivetrain;
    this.localization = localization;
    xError = new FilteredValue(() -> xController.getPositionError()).addMedianFilter(25);
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

public void setP(double newP)
{
 xController.setP(newP);
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

public void reset()
{
  thetaMeasurement = 0;
  xMeasurement = 0;
  xVelocity= 0;
  yVelocity = 0;
  botpose = new Pose2d();
  lastRobotPose2d = new Pose2d();
  lastRobotYaw = new Rotation2d();
  lastYaw = new Rotation2d();
}

public ChassisSpeeds calculateSpeeds(LimeLight ll, boolean hasCorrectTag)
{
    if(hasCorrectTag)
    {
      xVelocity = ll.config.getYawCos() * xController.calculate(getXMeasurement(), 0);
      rotationalVelocity = controller.calculate(getThetatMeasurement(), 0);
      double error = Math.abs(xError.calculate(xController.getPositionError()));
      double multiplier = 0;
      if(error < 10 && error >= 8)
      {
       multiplier = 0.25;  
      }
      else if(error < 8 && error >= 6)
      {
        multiplier = 0.5;
      }
      else if(error < 6 && error >= 4)
      {
        multiplier = 0.75;
      }
      else if(error < 4 && error >= 0)
      {
        multiplier = 2;
      }
     
      yVelocity = ll.config.getYawCos() * multiplier;
      System.out.println(error);
    }
    else
    {

        yVelocity = xController2.calculate(localization.getRelativePose().getX(),getTargetPoseRobotSpace().getX());
        xVelocity = xController2.calculate(localization.getRelativePose().getY(),getTargetPoseRobotSpace().getY());
        double rot = getLastRobotYaw().getDegrees() + getLastYaw().getDegrees();
        rotationalVelocity = -controller.calculate(MathUtil.angleModulus(drivetrain.getIMU().getYaw().getRadians()) * 180/Math.PI, rot);
      
    }
    speeds = new ChassisSpeeds(yVelocity, xVelocity, rotationalVelocity * 0.5);
    SmartDashboard.putNumberArray("Speeds",new Double[]{yVelocity, xVelocity, rotationalVelocity});
    return speeds;
}

}
