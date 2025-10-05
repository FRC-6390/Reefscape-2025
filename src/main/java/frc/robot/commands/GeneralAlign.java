package frc.robot.commands;

import java.security.PublicKey;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javax.print.attribute.standard.Media;

import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimeLightConfig;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateType;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatencyType;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.ReefScoringPos.ReefPole;

public class GeneralAlign {

    public int tagId = -1;
    public PIDController rController;
    public HolonomicDriveController controller;
    public boolean reached;
    public RobotBase<?> base;
    public LimeLight[] limelights;

    public GeneralAlign(RobotBase<?> base,PIDController rController, HolonomicDriveController controller, LimeLight... limelights)
    {
        this.base = base;
        this.limelights = limelights;
        this.rController = rController;
        this.controller = controller;
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

    public void reset()
    {
        tagId = -1;
        reached = false;   
    }
    
    public Pose2d getRobotPositionRelativeToTag(double limelightToFloorInches, double targetToFloorInches)
    {
        return getRobotPositionRelativeToTag(limelightToFloorInches, targetToFloorInches,false);
    }

    public Pose2d getRobotPositionRelativeToTag(double limelightToFloorInches, double targetToFloorInches, boolean shouldSetPose)
    {
    double x = 0;
    double y = 0;
    double count = 0;

    for (LimeLight limeLight : limelights) 
    {   
    double straigtLineDistToTag = limeLight.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9];
    double angleTX =  limeLight.getTargetHorizontalOffset() -base.getLocalization().getRelativePose().getRotation().getDegrees();
    double angleTY =  limeLight.getTargetVerticalOffset();
    double planeStraightDist = (Math.cos(Math.toRadians(angleTY)) * straigtLineDistToTag);

    double x1 = (Math.cos(Math.toRadians(angleTX)) * planeStraightDist) - limeLight.config.cameraRobotSpace().getX();
    double y1 = (Math.sin(Math.toRadians(angleTX)) * planeStraightDist)- limeLight.config.cameraRobotSpace().getY();

    if(limeLight.getAprilTagID() == tagId)
    {
    x += x1;
    y += y1;
    count += 1;
    }
    }

    x = x/count;
    y = y/count;
    

    Pose2d pose = new Pose2d(-x,y, base.getLocalization().getRelativePose().getRotation());
    base.getLocalization().resetRelativePose(pose);
    return pose;
    }

    public ChassisSpeeds calculateSpeeds(Pose2d generalPosition, Pose2d scoringPose)
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
