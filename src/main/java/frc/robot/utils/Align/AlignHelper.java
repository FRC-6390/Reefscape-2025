

package frc.robot.utils.Align;
 
import ca.frc6390.athena.core.RobotBase;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 
 public class AlignHelper {

  public enum AlignMode
  {
    LOOKAT,
    PARALLEL
  }


  public AlignCamera[] cams;

  public RobotBase<?> base;
  
  public int tagId = -1;
  public double targetMeasurement;
  public Pose2d finalPose2d;
  public PIDController rController;

  public HolonomicDriveController controller;
  public AlignMode mode = AlignMode.PARALLEL;



   public AlignHelper(int tagId, Pose2d targetPose2d ,RobotBase<?> base, PIDController rController, HolonomicDriveController controller, AlignMode mode, AlignCamera... cams)
   {
    this.cams = cams;
    this.base = base; 
    this.finalPose2d = targetPose2d;
    this.rController = rController;
    this.mode = mode;
    this.controller = controller;
    this.tagId = tagId;
   }

   public void init() 
   {
    rController.enableContinuousInput(-180, 180);
    rController.setTolerance(5);    
  }
  
   public Pose2d getPose2d()
   {
    double x = 0;
    double y = 0;
    double count = 0;

    for (AlignCamera alignCamera : cams) 
    {
      if(alignCamera.hasValidTarget() && alignCamera.getTagId() == tagId)
      {
      double dist = alignCamera.getDistanceToTag();
      //DANGER
      double dist1 = Math.cos(Math.toRadians(alignCamera.getTargetVerticalOffset())) * dist; 
      double angle1 =  alignCamera.getTargetHorizontalOffset() - alignCamera.getYaw() -base.getLocalization().getRelativePose().getRotation().getDegrees() ;
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
    Pose2d pose = new Pose2d(-xPos,yPos,base.getLocalization().getRelativePose().getRotation());
    return pose;
    }

  }

  public void setRelativePose()
  {
    if(getPose2d() != null)
    {
    base.getLocalization().resetRelativePose(getPose2d());
    }
  }

  public void shuffleboard()
  {
    SmartDashboard.putNumber("Field X", Units.metersToInches(getPose2d().getX()));
    SmartDashboard.putNumber("Field Y", Units.metersToInches(getPose2d().getY()));

    if(finalPose2d != null)
    {

    SmartDashboard.putNumber("Scoring Pos X", Units.metersToInches(finalPose2d.rotateBy(AprilTagMap.AprilTags.getRotation2d(tagId)).getX()));
    SmartDashboard.putNumber("Scoring Pos Y", Units.metersToInches(finalPose2d.rotateBy(AprilTagMap.AprilTags.getRotation2d(tagId)).getY()));
    SmartDashboard.putNumber("Tag ID", tagId);

    }
  }

  public void setGoal(Pose2d targetPose2d)
  {
    finalPose2d = targetPose2d;
  }

  public Pose2d getRobotPositionRelativeToTag()
  {
    return base.getLocalization().getRelativePose();
  }

  public double getDistanceToTag()
  {
    return base.getLocalization().getRelativePose().getTranslation().getDistance(new Translation2d());
  }

  public double getDistanceToTarget()
  {
    return base.getLocalization().getRelativePose().getTranslation().getDistance(finalPose2d.rotateBy(AprilTagMap.AprilTags.getRotation2d(tagId)).getTranslation().times(-1));
  }

  public Rotation2d getAngleToTag()
  {
    return base.getLocalization().getRelativePose().getTranslation().getAngle();
  }

  public Rotation2d getAngleToTarget()
  {
    Translation2d f = finalPose2d.rotateBy(AprilTagMap.AprilTags.getRotation2d(tagId)).getTranslation().times(-1);
    return Rotation2d.fromRadians(Math.atan2(f.getY() - base.getLocalization().getRelativePose().getTranslation().getY(), f.getX() - base.getLocalization().getRelativePose().getTranslation().getX()));
  }


  public ChassisSpeeds calculateSpeeds() 
  {
    controller.getXController().setP(1.75);
    controller.getYController().setP(1.75);

    double targetRot = 0;
    if(mode.equals(AlignMode.PARALLEL))
    {
      targetRot = AprilTagMap.AprilTags.getRotation2d(tagId).getDegrees();
    }
    else if(mode.equals(AlignMode.LOOKAT))
    {
      targetRot = Math.atan2(base.getLocalization().getRelativePose().getY(), base.getLocalization().getRelativePose().getX());
    }
    double rSpeed = rController.calculate(base.getLocalization().getRelativePose().getRotation().getDegrees(), targetRot);
    double xSpeed = controller.getXController().calculate(base.getLocalization().getRelativePose().getX(), finalPose2d.rotateBy(AprilTagMap.AprilTags.getRotation2d(tagId)).getX() * -1);
    double ySpeed = controller.getYController().calculate(base.getLocalization().getRelativePose().getY(), finalPose2d.rotateBy(AprilTagMap.AprilTags.getRotation2d(tagId)).getY() * -1);   
    
    ChassisSpeeds spds = ChassisSpeeds.fromFieldRelativeSpeeds
                                                (
                                                new ChassisSpeeds
                                                        (
                                                          xSpeed, 
                                                          ySpeed, 
                                                          rSpeed 
                                                        ), 
                                                base.getLocalization().getRelativePose().getRotation()
                                                );
    return spds;
  
 }
}