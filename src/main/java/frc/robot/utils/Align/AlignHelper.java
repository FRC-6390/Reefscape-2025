

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


  public AlignCamera[] cams;

  public Pose2d goalPose2d;
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



   public AlignHelper(RobotBase<?> base, PIDController rController, HolonomicDriveController controller, Pose2d goal, Pose2d scoring, AlignCamera... cams)
   {
    this.cams = cams;
    this.base = base; 
    this.goalPose2d = goal;
    this.finalPose2d = scoring;
    this.rController = rController;
    this.controller = controller;
    tagId = -1;
    isDone = false;
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
  
   public Pose2d getPose2d()
   {

    double x = 0;
    double y = 0;
    double count = 0;

    for (AlignCamera alignCamera : cams) 
    {
      LimeLight cam = alignCamera.getLimelight();
      if(cam.hasValidTarget())
      {
      double dist1 = cam.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9];
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

   public boolean closeEnough(String table)
    {
      LimeLight camera = base.getVision().getLimelight(table);
      return camera.hasValidTarget() && Math.abs(camera.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9]) <= 0.525 && Math.abs(camera.getTargetHorizontalOffset()) < 10;
    }

  public void setRelativePose()
  {
    base.getLocalization().resetRelativePose(getPose2d());
  }

  public ChassisSpeeds calculateSpeeds() 
  {

    double count = 0;
    for (AlignCamera alignCamera : cams) 
      {
        if(alignCamera.getLimelight().hasValidTarget())
        {
          thetaMeasurement += -filter.calculate(alignCamera.getLimelight().getPoseEstimate(PoseEstimateType.TARGET_POSE_ROBOT_SPACE).getRaw()[4]);
          count++;
        }   
    }
    
    thetaMeasurement = thetaMeasurement / count;

    targetMeasurement = base.getLocalization().getRelativePose().getRotation().getDegrees() - thetaMeasurement;

    if(tagId != -1)
    {

    if(base.getLocalization().getRelativePose().getTranslation().getDistance(goalPose2d.getTranslation()) > base.getLocalization().getRelativePose().getTranslation().getDistance(finalPose2d.getTranslation()))
    {
      reached = true;
    }

  
    if(base.getLocalization().getRelativePose().getTranslation().getDistance(goalPose2d.getTranslation()) > 0.075 && !reached)
    {  
    
    }
    else
    {
    reached = true;

    controller.getXController().setP(1.75);
    controller.getYController().setP(1.75);

    double rSpeed = rController.calculate(base.getLocalization().getRelativePose().getRotation().getDegrees(), AprilTagMap.AprilTags.getRotation2d(tagId).getDegrees());
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
    // base.getDrivetrain().getRobotSpeeds().setSpeeds("feedback", spds);
    return spds;
  }
  }
    return null;
 }}