

package frc.robot.utils.Align;
 
import ca.frc6390.athena.controllers.DelayedOutput;
 import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
 import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateType;
 import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatencyType;

import static edu.wpi.first.units.Units.Rotation;

import java.text.NumberFormat.Style;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
 import edu.wpi.first.math.controller.ProfiledPIDController;
 import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryGenerator.ControlVectorList;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
 
 public class AlignHelper {

  public enum AlignMode
  {
    LOOKAT,
    PARALLEL
  }


  public AlignCamera[] cams;

  public RobotBase<?> base;
  public boolean reached = false;
  public boolean rightPole = false;
  public double thetaMeasurement = 0;
  
  public int tagId = -1;
  public MedianFilter filter;
  public double targetMeasurement;
  public Pose2d finalPose2d;
  public PIDController rController;

  public HolonomicDriveController controller;
  public boolean isDone = false;
  public AlignMode mode = AlignMode.PARALLEL;



   public AlignHelper(RobotBase<?> base, PIDController rController, HolonomicDriveController controller, AlignMode mode, AlignCamera... cams)
   {
    this.cams = cams;
    this.base = base; 
    this.rController = rController;
    this.mode = mode;
    this.controller = controller;
    tagId = -1;
    isDone = false;
   }


  public void setGoal(Pose2d scoring)
  {
  this.finalPose2d = scoring;
  }

   public void init() 
   {
    thetaMeasurement = 0;
    rController.enableContinuousInput(-180, 180);
    rController.setTolerance(5);

    isDone = false;
    filter = new MedianFilter(50);
    tagId = -1;

    reached = false;
    
  }

   public static int findMostFrequent(List<Integer> nums) 
    {
        
    Map<Integer, Integer> frequencyMap = new HashMap<>();
    for (int num : nums) 
    {
        frequencyMap.put(num, frequencyMap.getOrDefault(num, 0) + 1);
    }

    int maxCount = 0;
    int mostFrequent = nums.get(0);
    for (Map.Entry<Integer, Integer> entry : frequencyMap.entrySet()) 
    {
        if (entry.getValue() > maxCount || (entry.getValue() == maxCount && entry.getKey() > mostFrequent)) 
            {
            maxCount = entry.getValue();
            mostFrequent = entry.getKey();
            }
    }
    return mostFrequent;
    }
 

  
  public void setId()
  {
    if(tagId == -1)
    {
    List<Integer> ids = new ArrayList<>();

    for (AlignCamera cam : cams)
        {
            LimeLight limelight = cam.getLimelight();
            if(limelight.hasValidTarget())
            {
                ids.add((int)limelight.getAprilTagID());
            }
        }
        
        if(tagId == -1 && !ids.isEmpty())
        {
        tagId = findMostFrequent(ids);
        }
    }
   
  }

  public void setId(int id)
  {
    tagId = id;
  }
  
   public Pose2d getPose2d()
   {

    double x = 0;
    double y = 0;
    double count = 0;

    for (AlignCamera alignCamera : cams) 
    {
      LimeLight cam = alignCamera.getLimelight();
      if(cam.hasValidTarget() && cam.getAprilTagID() == tagId)
      {
      double dist = cam.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9];
      //DANGER
      double dist1 = Math.cos(Math.toRadians(cam.getTargetVerticalOffset())) * dist; 
      double angle1 =  cam.getTargetHorizontalOffset() - alignCamera.getYaw() -base.getLocalization().getRelativePose().getRotation().getDegrees() ;
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
     
    SmartDashboard.putNumber("Field X", Units.metersToInches(pose.getX()));
    SmartDashboard.putNumber("Field Y", Units.metersToInches(pose.getY()));

    if(finalPose2d != null)
    {

    SmartDashboard.putNumber("Scoring Pos X", Units.metersToInches(finalPose2d.rotateBy(AprilTagMap.AprilTags.getRotation2d(tagId)).getX()));
    SmartDashboard.putNumber("Scoring Pos Y", Units.metersToInches(finalPose2d.rotateBy(AprilTagMap.AprilTags.getRotation2d(tagId)).getY()));
    SmartDashboard.putNumber("Tag ID", tagId);

    }

   
    return pose;
    }
   }

   public boolean closeEnough(String table)
    {
      LimeLight camera = base.getVision().getLimelight(table);
      return camera.hasValidTarget() && Math.abs(camera.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9]) <= 0.525 && Math.abs(camera.getTargetHorizontalOffset()) < 10;
    }

  public void setRelativePose()
  {
    if(getPose2d() != null)
    {
    base.getLocalization().resetRelativePose(getPose2d());
    }
  }

  public Pose2d getRobotPositionRelativeToTag()
  {
    return base.getLocalization().getRelativePose();
  }

  public double getDistanceToTag()
  {
    return base.getLocalization().getRelativePose().getTranslation().getDistance(new Translation2d());
  }

  public Rotation2d getAngleToTag()
  {
    return base.getLocalization().getRelativePose().getTranslation().getAngle();
  }


  public ChassisSpeeds calculateSpeeds() 
  {
    if(tagId != -1)
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
  
    return null;
 }}