

package frc.robot.utils.Align;
 
import ca.frc6390.athena.core.RobotCore;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 
 public class RelevantPosition extends SubsystemBase {

  public enum AlignMode
  {
    LOOKAT,
    PARALLEL
  }


  private AutomationCamera[] cams;

  public RobotCore<?> base;
  
  private int tagId = -1;
  private AprilTag tag;
  private double targetMeasurement;
  private Pose2d finalPose2d;

  private String name;


   public RelevantPosition(String name,AprilTag tag, Pose2d targetPose2d ,RobotCore<?> base, AutomationCamera... cams)
   {
    this.cams = cams;
    this.base = base; 
    this.finalPose2d = targetPose2d;
    this.tag = tag;
    this.tagId = (int) tag.getId();
    this.name = name;
    // this.base.getLocalization().(name, () -> this.base.getLocalization().(name));
   }

  
   public Pose2d getPose2d()
   {
    double x = 0;
    double y = 0;
    double count = 0;

    for (AutomationCamera alignCamera : cams) 
    {
      if(alignCamera.hasValidTarget() && alignCamera.getTagId() == tagId)
      {
      double dist = alignCamera.getDistanceToTag();
      double dist1 = Math.cos(Math.toRadians(alignCamera.getTargetVerticalOffset() + alignCamera.getPitch())) * dist; 
      double angle1 =  alignCamera.getTargetHorizontalOffset() - alignCamera.getYaw() -base.getLocalization().getPose2d(name).getRotation().getDegrees() ;
      double x1 = (Math.cos(Math.toRadians(angle1)) * dist1) - alignCamera.getXOffset();
      double y1 = (Math.sin(Math.toRadians(angle1)) * dist1)- alignCamera.getYOffset(); 

      x += x1;
      y += y1;
      count += 1;
      }
    }

    double xPos = x / count;
    double yPos = y / count;
    if(Double.isNaN(xPos) || Double.isNaN(yPos))
    {
    return null;
    }
    else
    {
    Pose2d pose = new Pose2d(-xPos,yPos,base.getLocalization().getPose2d(name).getRotation());
    return pose;
    }

  }

  public void setRelativePose()
  {
    if(getPose2d() != null)
    {
    base.getLocalization().resetPose(name, getPose2d());
    }
  }

  public void initFieldRelativeStartingPose(Pose2d fieldRelativeStartingPose)
  {
    base.getLocalization().resetPose(name, new Pose2d(tag.getPose2d().minus(fieldRelativeStartingPose).getX(),tag.getPose2d().minus(fieldRelativeStartingPose).getY(), fieldRelativeStartingPose.getRotation()));
  }

  public void shuffleboard()
  {
    SmartDashboard.putNumber("Field X", Units.metersToInches(getPose2d().getX()));
    SmartDashboard.putNumber("Field Y", Units.metersToInches(getPose2d().getY()));

    if(finalPose2d != null)
    {

    SmartDashboard.putNumber("Scoring Pos X", Units.metersToInches(finalPose2d.rotateBy(tag.getRotation2d()).getX()));
    SmartDashboard.putNumber("Scoring Pos Y", Units.metersToInches(finalPose2d.rotateBy(tag.getRotation2d()).getY()));
    SmartDashboard.putNumber("Tag ID", tagId);

    }
  }

  public void update()
  {
    setRelativePose();
    shuffleboard();
  }
  public void setGoal(Pose2d targetPose2d)
  {
    finalPose2d = targetPose2d;
  }

  public Pose2d getRobotPositionRelativeToTag()
  {
    return base.getLocalization().getPose2d(name);
  }

  public double getDistanceToTag()
  {
    return base.getLocalization().getPose2d(name).getTranslation().getDistance(new Translation2d());
  }

  public double getDistanceToTarget()
  {
    return base.getLocalization().getPose2d(name).getTranslation().getDistance(finalPose2d.rotateBy(tag.getRotation2d()).getTranslation().times(-1));
  }

  public Rotation2d getAngleToTag()
  {
    return base.getLocalization().getPose2d(name).getTranslation().getAngle();
  }

  public Rotation2d getAngleToTarget()
  {
    Translation2d f = finalPose2d.rotateBy(tag.getRotation2d()).getTranslation().times(-1);
    return Rotation2d.fromRadians(Math.atan2(f.getY() - base.getLocalization().getPose2d(name).getTranslation().getY(), f.getX() - base.getLocalization().getPose2d(name).getTranslation().getX()));
  }


  public DriveToPoint driveTo(PIDController rController, PIDController xController, PIDController yController)
  {
    return new DriveToPoint(this, rController, xController, yController);
  }


  public ChassisSpeeds calculateSpeeds(PIDController rController, PIDController xController, PIDController yController, AlignMode mode) 
  {
    if(mode.equals(AlignMode.PARALLEL))
    {
      targetMeasurement = tag.getRotation2d().getDegrees();
    }
    else if(mode.equals(AlignMode.LOOKAT))
    {
      targetMeasurement = Math.atan2(base.getLocalization().getPose2d(name).getY(), base.getLocalization().getPose2d(name).getX());
    }
    double rSpeed = rController.calculate(base.getLocalization().getPose2d(name).getRotation().getDegrees(), targetMeasurement);
    double xSpeed = xController.calculate(base.getLocalization().getPose2d(name).getX(), finalPose2d.rotateBy(tag.getRotation2d()).getX() * -1);
    double ySpeed = yController.calculate(base.getLocalization().getPose2d(name).getY(), finalPose2d.rotateBy(tag.getRotation2d()).getY() * -1);   
    
    ChassisSpeeds spds = ChassisSpeeds.fromFieldRelativeSpeeds
                                                (
                                                new ChassisSpeeds
                                                        (
                                                          xSpeed, 
                                                          ySpeed, 
                                                          rSpeed 
                                                        ), 
                                                base.getLocalization().getPose2d(name).getRotation()
                                                );
    return spds;
  
 }

 @Override
 public void periodic() {
     update();
 }
}