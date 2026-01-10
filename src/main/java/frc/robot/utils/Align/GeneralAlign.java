package frc.robot.utils.Align;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatencyType;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.utils.Align.AprilTagMap.AprilTags;

public class GeneralAlign {

    public int tagId = -1;
    public PIDController rController;
    public HolonomicDriveController controller;
    public boolean reached;
    public RobotBase<?> base;
    public AlignCamera[] limelights;
    public Pose2d generalPosition = new Pose2d();
    public Pose2d scoringPose = new Pose2d();
    public ShuffleboardTab tab;

    public GeneralAlign(RobotBase<?> base,PIDController rController, HolonomicDriveController controller, Pose2d generalPosition, Pose2d scoringPose2d, AlignCamera... limelights)
    {
        this.base = base;
        this.limelights = limelights;
        this.rController = rController;
        this.controller = controller;
        // this.generalPosition = generalPosition;
        // this.scoringPose = scoringPose2d;
        tab = Shuffleboard.getTab("Auto Align");
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
    
    public int getTagId()
    {
        return tagId;
    }

    public void shuffleboard()
    {
        // tab.addNumber("X Controller Auto Align", () -> controller.getXController());
        // tab.add("Y Controller Auto Align", controller.getYController());

        // tab.add("Rotation Controller Auto Align", rController);
       
        Pose2d general = convertToFieldRelativePose(generalPosition);
        Pose2d scoring = convertToFieldRelativePose(scoringPose);


        tab.addNumber("X Field", () -> Units.metersToInches(getPose2d(limelights).getX()));
        tab.addNumber("Y Field", () -> Units.metersToInches(getPose2d(limelights).getY()));

        tab.addNumber("X Tag", () -> Units.metersToInches(convertToTagRelativePose(getPose2d(limelights)).getX()));
        tab.addNumber("Y Tag", () -> Units.metersToInches(convertToTagRelativePose(getPose2d(limelights)).getY()));

        tab.addNumber("X Scoring Tag", () -> Units.metersToInches(scoringPose.getX()));
        tab.addNumber("Y Scoring Tag", () -> Units.metersToInches(scoringPose.getY()));

        tab.addNumber("X Scoring Field", () -> Units.metersToInches(scoring.getX()));
        tab.addNumber("Y Scoring Field", () -> Units.metersToInches(scoring.getY()));
        
    }

    public Pose2d convertToFieldRelativePose(Pose2d tagRelativePose)
    {
        if(tagRelativePose != null)
        {
        return tagRelativePose.rotateBy(AprilTagMap.AprilTags.getRotation2d(tagId));
        }
        else
        {
        return new Pose2d();
        }

    }

    public Pose2d convertToTagRelativePose(Pose2d field)
    {
        if(field != null)
        {
        return field.rotateBy(Rotation2d.fromDegrees(AprilTagMap.AprilTags.getRotation2d(tagId).getDegrees() * -1));
        }
        else
        {
        return new Pose2d();
        }
    }
    
    public void setUp()
    {
        List<Integer> ids = new ArrayList<>();

        for (AlignCamera cam : limelights)
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

        // base.getLocalization().resetRelativePose(new Pose2d(0,0,Rotation2d.fromDegrees(base.getLocalization().getRelativePose().getRotation().getDegrees() - AprilTagMap.AprilTags.getRotation2d(tagId).getDegrees())));
    }


    //Resets the autoalign
    public void reset()
    {
        tagId = -1;
        reached = false;   
    }

    //Set the localization relative pose of the robot
    public void updateBotpose()
    {
        Pose2d t = getPose2d(limelights);
        base.getLocalization().resetRelativePose(t.getX(), t.getY());
    }

    public Pose2d getPose2d(AlignCamera... cams)
   {
    double count = cams.length;
    double totalX = 0;
    double totalY = 0;

    for (AlignCamera alignCamera : cams) 
    {
      if(alignCamera.getLimelight().getAprilTagID() == tagId)
      {
        LimeLight camera = base.getVision().getLimelight("limelight-left");
        double dist = camera.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9];
        double angle =  camera.getTargetHorizontalOffset() -base.getLocalization().getRelativePose().getRotation().getDegrees() ;
        double x = (Math.cos(Math.toRadians(angle + alignCamera.getYaw())) * dist) - Units.inchesToMeters(0.5);
        double y = (Math.sin(Math.toRadians(angle + alignCamera.getYaw())) * dist)- Units.inchesToMeters(9.25);
        totalX += x;
        totalY += y;
        count++;
      }  
    }

    double x = totalX / count;
    double y = totalY / count;

    Pose2d pose = new Pose2d(-x,y,base.getLocalization().getRelativePose().getRotation());
    
    return pose;
   }
   
    public void setGeneralPosition(Pose2d pose)
    {
        generalPosition = pose;
    }

    public void setScoringPos(Pose2d pose)
    {
        scoringPose = pose;
    }

    //Calculating the speed to the robot
    public ChassisSpeeds calculateSpeeds()
    {
    Pose2d general = convertToFieldRelativePose(generalPosition);
    Pose2d scoring = convertToFieldRelativePose(scoringPose);
    boolean preciseMode = (base.getLocalization().getRelativePose().getTranslation().getDistance(generalPosition.getTranslation()) > 0.075 && !reached);
    

    double rSpeed = rController.calculate(base.getLocalization().getRelativePose().getRotation().getDegrees(), preciseMode ? general.getRotation().getDegrees() : scoring.getRotation().getDegrees());
    double xSpeed = controller.getXController().calculate(base.getLocalization().getRelativePose().getX(), preciseMode ? general.getX() : scoring.getX());
    double ySpeed = controller.getYController().calculate(base.getLocalization().getRelativePose().getY(), preciseMode ? general.getY() : scoring.getY());

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
