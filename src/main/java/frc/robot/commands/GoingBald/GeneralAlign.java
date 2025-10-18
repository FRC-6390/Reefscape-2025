package frc.robot.commands.GoingBald;

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
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.GoingBald.AprilTagMap.AprilTags;

public class GeneralAlign {

    public int tagId = -1;
    public PIDController rController;
    public HolonomicDriveController controller;
    public boolean reached;
    public RobotBase<?> base;
    public AlignCamera[] limelights;
    public Pose2d generalPosition;
    public Pose2d scoringPose;
    public ShuffleboardTab tab;

    public GeneralAlign(RobotBase<?> base,PIDController rController, HolonomicDriveController controller, Pose2d generalPosition, Pose2d scoringPose2d, AlignCamera... limelights)
    {
        this.base = base;
        this.limelights = limelights;
        this.rController = rController;
        this.controller = controller;
        this.generalPosition = generalPosition;
        this.scoringPose = scoringPose2d;
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
        tab.addNumber("Tag Id", () -> getTagId());
        tab.addNumber("X Relative Pose From Local (Field Relative Pose)", () -> Units.metersToInches(base.getLocalization().getRelativePose().getX()));
        tab.addNumber("Y Relative Pose From Local (Field Relative Pose)", () -> Units.metersToInches(base.getLocalization().getRelativePose().getY()));
        tab.addNumber("StraightLine Distance Left", () -> Units.metersToInches(base.getVision().getLimelight("limelight-left").getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9]));
        tab.addNumber("StraightLine Distance Right", () -> Units.metersToInches(base.getVision().getLimelight("limelight-right").getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9]));


        tab.addNumber("X Tag Relative Pose", () -> Units.metersToInches(getRobotPositionRelativeToTag().getX()));
        tab.addNumber("Y Tag Relative Pose", () -> Units.metersToInches(getRobotPositionRelativeToTag().getY()));

    }
    
    //Chooses the tag that most limelights see
    public int setTagId()
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
        
        if(tagId == -1)
        {
        tagId = findMostFrequent(ids);
        }
        return tagId;
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
        base.getLocalization().resetRelativePose(convertToFieldRelativeAxises(getRobotPositionRelativeToTag(), tagId));
    }
   
   //Converts tag relative position to field relative position
   public Pose2d convertToFieldRelativeAxises(Pose2d tagRelativePose, long id) 
   {
    Rotation2d tagRotation = AprilTags.getRotation2d(id);

    Translation2d rotatedTranslation = tagRelativePose.getTranslation().rotateBy(tagRotation);

    Rotation2d robotRotation = tagRotation.plus(tagRelativePose.getRotation());

    return new Pose2d(rotatedTranslation, robotRotation);
   }

   //Gets tag relative translation
   public Translation2d getTagRelativeTranslation(AlignCamera cam)
   {
    LimeLight limeLight = cam.getLimelight();
    double straigtLineDistToTag = limeLight.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9];
    double angleTX =  limeLight.getTargetHorizontalOffset();
    double angleTY =  limeLight.getTargetVerticalOffset();
    double planeStraightDist = (Math.cos(Math.toRadians(angleTY)) * straigtLineDistToTag);

    double x1 = (Math.cos(Math.toRadians(angleTX)) * planeStraightDist) +  cam.getXOffset();
    double y1 = (Math.sin(Math.toRadians(angleTX)) * planeStraightDist) + cam.getYOffset();

    return new Translation2d(x1, y1);
   }

    //Averages all the positions calculated by each limelight
    public Pose2d getRobotPositionRelativeToTag()
    {
    double x = 0;
    double y = 0;
    double count = 0;

    for (AlignCamera cam : limelights) 
    {  
    LimeLight limeLight = cam.getLimelight(); 
    System.out.println(limeLight.config.getTable());
    if(limeLight.getAprilTagID() == tagId)
        {
            x += getTagRelativeTranslation(cam).getX();
            y += getTagRelativeTranslation(cam).getY();
            count += 1;
        }
    }

    x = x/count;
    y = y/count;
    
    Pose2d pose = new Pose2d(-x,y, base.getLocalization().getRelativePose().getRotation());
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
    boolean preciseMode = (base.getLocalization().getRelativePose().getTranslation().getDistance(generalPosition.getTranslation()) > 0.075 && !reached);
  
    double rSpeed = rController.calculate(base.getLocalization().getRelativePose().getRotation().getDegrees(), preciseMode ? generalPosition.getRotation().getDegrees() : scoringPose.getRotation().getDegrees());
    double xSpeed = controller.getXController().calculate(base.getLocalization().getRelativePose().getX(), preciseMode ? generalPosition.getX() : scoringPose.getX());
    double ySpeed = controller.getYController().calculate(base.getLocalization().getRelativePose().getY(), preciseMode ? generalPosition.getY() : scoringPose.getY());

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
