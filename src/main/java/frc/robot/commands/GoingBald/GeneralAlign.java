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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.GoingBald.AprilTagMap.AprilTags;

public class GeneralAlign {

    public int tagId = -1;
    public PIDController rController;
    public HolonomicDriveController controller;
    public boolean reached;
    public RobotBase<?> base;
    public LimeLight[] limelights;
    public Pose2d generalPosition;
    public Pose2d scoringPose;
    public ShuffleboardTab tab;

    GeneralAlign(RobotBase<?> base,PIDController rController, HolonomicDriveController controller, Pose2d generalPosition, Pose2d scoringPose2d, LimeLight... limelights)
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
        tab.add("Controller", controller);
        tab.add("Rotation Controller", rController);
        tab.add("Tag Id", getTagId());
        tab.add("Relative Pose From Local (Field Relative Pose)", base.getLocalization().getRelativePose());
        tab.add("Tag Relative Pose", getRobotPositionRelativeToTag(0, 0));
    }
    
    //Chooses the tag that most limelights see
    public int setTagId()
    {
        List<Integer> ids = new ArrayList<>();

        for (LimeLight limelight : limelights)
        {
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
        base.getLocalization().resetRelativePose(convertToFieldRelativeAxises(getRobotPositionRelativeToTag(0, 0), tagId));
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
   public Translation2d getTagRelativeTranslation(LimeLight limeLight)
   {
    double straigtLineDistToTag = limeLight.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9];
    double angleTX =  limeLight.getTargetHorizontalOffset();
    double angleTY =  limeLight.getTargetVerticalOffset();
    double planeStraightDist = (Math.cos(Math.toRadians(angleTY)) * straigtLineDistToTag);

    double x1 = (Math.cos(Math.toRadians(angleTX)) * planeStraightDist) - limeLight.config.cameraRobotSpace().getX();
    double y1 = (Math.sin(Math.toRadians(angleTX)) * planeStraightDist)- limeLight.config.cameraRobotSpace().getY();

    return new Translation2d(x1, y1);
   }

    //Averages all the positions calculated by each limelight
    public Pose2d getRobotPositionRelativeToTag(double limelightToFloorInches, double targetToFloorInches)
    {
    double x = 0;
    double y = 0;
    double count = 0;

    for (LimeLight limeLight : limelights) 
    {   
    if(limeLight.getAprilTagID() == tagId)
        {
            x += getTagRelativeTranslation(limeLight).getX();
            y += getTagRelativeTranslation(limeLight).getY();
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
