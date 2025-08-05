

package frc.robot.commands.auto;
 
import ca.frc6390.athena.controllers.DelayedOutput;
 import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
 import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateType;
 import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatencyType;

import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
 import edu.wpi.first.math.controller.ProfiledPIDController;
 import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryGenerator.ControlVectorList;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
 import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.ReefScoringPos.ReefPole;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
 
 public class V2 extends Command {

  public LimeLight ll;
  public LimeLight lr;
  public RobotSpeeds robotSpeeds;
  public Pose2d goalPose2d = new Pose2d(Units.inchesToMeters(25), Units.inchesToMeters(6.5),new Rotation2d());

  public RobotBase<?> base;
  public boolean reached = false;
  public double thetaMeasurement = 0;
  public int tagId = -1;
  public MedianFilter filter;
  public Trajectory trajectory; 
  public double targetMeasurement;
  public double startTime;
  public Pose2d finalPose2d = new Pose2d(Units.inchesToMeters(5), Units.inchesToMeters(6.5),new Rotation2d());
  public PIDController rController = new PIDController(0.04, 0, 0);

  public HolonomicDriveController controller = new HolonomicDriveController(
                                                          new PIDController(1, 0, 0), 
                                                          new PIDController(1, 0, 0),
                                                          new ProfiledPIDController(0, 0, 0, new Constraints(0, 0)));
  public boolean isDone = false;



   public V2(RobotBase<?> base,String lltable)
   {
    lr = base.getVision().getLimelight("limelight-right");
    ll = base.getVision().getLimelight("limelight-left");
    this.base = base;
    robotSpeeds = base.getRobotSpeeds();
    tagId = -1;
    isDone = false;
   }

   @Override
   public void initialize() 
   {
    thetaMeasurement = 0;
    isDone = false;
    filter = new MedianFilter(50);
    tagId = -1;

    reached = false;
    if(ll.hasValidTarget())
    {
      base.getLocalization()
      .resetFieldPose
      (
        getPose2d()
      );
      tagId = ((int)ll.getAprilTagID());
    } 
    else if(lr.hasValidTarget())
    {
      base.getLocalization()
      .resetFieldPose
      (
        getPose2d()
      );
      tagId = ((int)lr.getAprilTagID());
    } 

    // trajectory = TrajectoryGenerator.generateTrajectory(List.of(base.getLocalization().getFieldPose(), new Pose2d(0, 0, new Rotation2d())), new TrajectoryConfig(4, 4));

  }
 

  
  
   public Pose2d getPose2d()
   {

    LimeLight ll1 = base.getVision().getLimelight("limelight-left");
    double dist1 = ll1.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9];
    double angle1 =  ll1.getTargetHorizontalOffset() -base.getLocalization().getFieldPose().getRotation().getDegrees() ;
    double x1 = (Math.cos(Math.toRadians(angle1)) * dist1) - Units.inchesToMeters(5);
    double y1 = (Math.sin(Math.toRadians(angle1)) * dist1)- Units.inchesToMeters(10);; 

    
     LimeLight ll = base.getVision().getLimelight("limelight-right");
     double dist = ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9];
     double angle =  ll.getTargetHorizontalOffset() -base.getLocalization().getFieldPose().getRotation().getDegrees() ;
     double x2 = (Math.cos(Math.toRadians(angle)) * dist) - Units.inchesToMeters(5);
     double y2 = (Math.sin(Math.toRadians(angle)) * dist) + Units.inchesToMeters(10);
     double x = 0;
     double y = 0;
     if(ll1.hasValidTarget() && ll.hasValidTarget())
     {
       x = (x2 + x1)/2;
       y = (y2 + y1)/2;
     }
     else if(!ll1.hasValidTarget() && ll.hasValidTarget())
     {
       x = (x2);
       y = (y2);
     }
     else if(ll1.hasValidTarget() && !ll.hasValidTarget())
     {
       x = (x1);
       y = (y1);
     }

     Pose2d pose = new Pose2d(-x,y,base.getLocalization().getFieldPose().getRotation());
     if(ll.hasValidTarget())
     {
      Pose2d pole = ReefPole.getPoleFromID(ll.getAprilTagID(), ll).getPose2d();
      base.getLocalization().getField2dObject("flipped").setPose(new Pose2d(pose.getY()+pole.getX(), pose.getX()+pole.getY(), pose.getRotation().plus(pole.getRotation())));
     base.getLocalization().getField2dObject("RobotPose").setPose(pose.relativeTo(ReefPole.getPoleFromID(ll.getAprilTagID(), ll).getPose2d()));
     base.getLocalization().getField2dObject("Tag").setPose(ReefPole.getPoleFromID(ll.getAprilTagID(), ll).getPose2d());
     }
     else if(lr.hasValidTarget())
     {
      Pose2d pole = ReefPole.getPoleFromID(lr.getAprilTagID(), lr).getPose2d();
      base.getLocalization().getField2dObject("flipped").setPose(new Pose2d(pose.getY()+pole.getX(), pose.getX()+pole.getY(), pose.getRotation().plus(pole.getRotation())));    
      base.getLocalization().getField2dObject("RobotPose").setPose(pose.relativeTo(ReefPole.getPoleFromID(lr.getAprilTagID(), lr).getPose2d()));
     base.getLocalization().getField2dObject("Tag").setPose(ReefPole.getPoleFromID(lr.getAprilTagID(), lr).getPose2d());
     }
     return pose;
   }

   public boolean closeEnough(String table)
    {
      LimeLight ll = base.getVision().getLimelight(table);
      return ll.hasValidTarget() && ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9] <= 0.525 && Math.abs(ll.getTargetHorizontalOffset()) < 5;
    }

 
   // Called every time the scheduler runs while the command is scheduled.
   @Override
   public void execute() 
   {

    if(ll.hasValidTarget() || lr.hasValidTarget())
    {
      thetaMeasurement =-filter.calculate(ll.getPoseEstimate(PoseEstimateType.TARGET_POSE_ROBOT_SPACE).getRaw()[4]);
      if(tagId == -1)
      {
        if(ll.hasValidTarget())
        {
          tagId = (int)ll.getAprilTagID();
        }
        if(lr.hasValidTarget())
        {
          tagId = (int)lr.getAprilTagID();
        }
      }
      if(ll.hasValidTarget() && (int)ll.getAprilTagID() == tagId)
      {
      goalPose2d = new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(6),new Rotation2d()).rotateAround(new Translation2d(), ReefPole.getPoleFromID(ll.getAprilTagID(), ll).getRotation());
      finalPose2d = new Pose2d(Units.inchesToMeters(9), Units.inchesToMeters(10),new Rotation2d()).rotateAround(new Translation2d(), ReefPole.getPoleFromID(ll.getAprilTagID(), ll).getRotation());
      }
      else if(!ll.hasValidTarget() || (int)ll.getAprilTagID() != tagId)
      {
        goalPose2d = base.getLocalization().getFieldPose();
        finalPose2d = base.getLocalization().getFieldPose();

      }
      if(lr.hasValidTarget() && (int)lr.getAprilTagID() == tagId)
      {
      goalPose2d = new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(6),new Rotation2d()).rotateAround(new Translation2d(), ReefPole.getPoleFromID(lr.getAprilTagID(), lr).getRotation());
      finalPose2d = new Pose2d(Units.inchesToMeters(9), Units.inchesToMeters(10),new Rotation2d()).rotateAround(new Translation2d(), ReefPole.getPoleFromID(lr.getAprilTagID(), lr).getRotation());
      }
      else if(!lr.hasValidTarget() || (int)lr.getAprilTagID() != tagId)
      {
        goalPose2d = base.getLocalization().getFieldPose();
        finalPose2d = base.getLocalization().getFieldPose();

      }
    
      base.getLocalization()
      .resetFieldPose
      (
        getPose2d()
      );

      targetMeasurement = base.getLocalization().getFieldPose().getRotation().getDegrees() - thetaMeasurement;

    }

    if(closeEnough("limelight-left") || closeEnough("limelight-right"))
    {
      isDone = true;
    }

    if(tagId != -1)
    {

    if(base.getLocalization().getFieldPose().getTranslation().getDistance(goalPose2d.getTranslation()) > 0.075 && !reached)
    {  
    controller.getXController().setP(2);
    controller.getYController().setP(2);

  
    double rSpeed = rController.calculate(base.getLocalization().getFieldPose().getRotation().getDegrees(), ReefPole.getPoleFromID(tagId, ll).getRotation().getDegrees() - 180);
   
    double xSpeed = controller.getXController().calculate(base.getLocalization().getFieldPose().getX(), goalPose2d.getX());
    double ySpeed = controller.getYController().calculate(base.getLocalization().getFieldPose().getY(), goalPose2d.getY());
    
    base.getDrivetrain().getRobotSpeeds().setSpeeds("feedback", new ChassisSpeeds(xSpeed, ySpeed, rSpeed));
    // base.getDrivetrain().getRobotSpeeds().setSpeeds("feedback", new ChassisSpeeds(0, 0, rSpeed));

    }
    else
    {
    reached = true;
    double rSpeed = rController.calculate(base.getLocalization().getFieldPose().getRotation().getDegrees(), ReefPole.getPoleFromID(tagId, ll).getRotation().getDegrees() - 180);
    
    controller.getXController().setP(1.25);
    controller.getYController().setP(1.25);
    double xSpeed = controller.getXController().calculate(base.getLocalization().getFieldPose().getX(), finalPose2d.getX());
    double ySpeed = controller.getYController().calculate(base.getLocalization().getFieldPose().getY(), finalPose2d.getY());
    
    base.getDrivetrain().getRobotSpeeds().setSpeeds("feedback", new ChassisSpeeds(xSpeed, ySpeed, rSpeed));
        // base.getDrivetrain().getRobotSpeeds().setSpeeds("feedback", new ChassisSpeeds(0, 0, rSpeed));

    }
  }
   }
 
 
   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {
     
     base.getDrivetrain().getRobotSpeeds().setSpeeds("feedback", new ChassisSpeeds(0,0,0));

   }
 
   // Returns true when the command should end.
   @Override
   public boolean isFinished() {
     
     return isDone;
   }
 }