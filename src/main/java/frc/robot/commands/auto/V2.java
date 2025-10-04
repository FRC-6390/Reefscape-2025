

package frc.robot.commands.auto;
 
import ca.frc6390.athena.controllers.DelayedOutput;
 import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
 import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateType;
 import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatencyType;

import java.util.List;
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
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.utils.ReefScoringPos.ReefPole;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
 
 public class V2 extends Command {


  public LimeLight camera_left;
  public LimeLight camera_right;
  public Pose2d goalPose2d = new Pose2d(Units.inchesToMeters(25), Units.inchesToMeters(6.5),new Rotation2d());

  public RobotBase<?> base;
  public boolean reached = false;
  public boolean rightPole = false;
  public double thetaMeasurement = 0;
  
  public Superstructure superstructure;
  public Supplier<SuperstructureState> state;
  public int tagId = -1;
  public MedianFilter filter;
  public double targetMeasurement;
  public Pose2d finalPose2d = new Pose2d(Units.inchesToMeters(5), Units.inchesToMeters(6.5),new Rotation2d());
  public PIDController rController = new PIDController(0.07, 0, 0);
  //0.04

  public HolonomicDriveController controller = new HolonomicDriveController(
                                                          new PIDController(1, 0, 0), 
                                                          new PIDController(1, 0, 0),
                                                          new ProfiledPIDController(0, 0, 0, new Constraints(0, 0)));
  public boolean isDone = false;



   public V2(RobotBase<?> base,String lltable, boolean rightPole, Superstructure superstructure, Supplier<SuperstructureState> state)
   {
    camera_right = base.getVision().getLimelight("limelight-right");
    camera_left = base.getVision().getLimelight("limelight-left");
    this.superstructure = superstructure;
    this.state = state;
    this.base = base;
    this.rightPole = rightPole; 
    tagId = -1;
    isDone = false;
   }

   @Override
   public void initialize() 
   {
    thetaMeasurement = 0;
    rController.enableContinuousInput(-180, 180);
    rController.setTolerance(5);

    isDone = false;
    filter = new MedianFilter(50);
    tagId = -1;

    reached = false;
    if(camera_left.hasValidTarget())
    {
      // base.getLocalization()
      // .resetRelativePose
      // (
      //   getPose2d()
      // );
      tagId = ((int)camera_left.getAprilTagID());
    } 
    else if(camera_right.hasValidTarget())
    {
      // base.getLocalization()
      // .resetRelativePose
      // (
      //   getPose2d()
      // );
      tagId = ((int)camera_right.getAprilTagID());
    } 
  }
 

  
  
   public Pose2d getPose2d()
   {

    LimeLight camera_left = base.getVision().getLimelight("limelight-left");
    double dist1 = camera_left.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9];
    double angle1 =  camera_left.getTargetHorizontalOffset() -base.getLocalization().getRelativePose().getRotation().getDegrees() ;
    double x1 = (Math.cos(Math.toRadians(angle1)) * dist1) - Units.inchesToMeters(5);
    double y1 = (Math.sin(Math.toRadians(angle1)) * dist1)- Units.inchesToMeters(10);; 

    
     LimeLight camera_right = base.getVision().getLimelight("limelight-right");
     double dist = camera_right.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9];
     double angle =  camera_right.getTargetHorizontalOffset() -base.getLocalization().getRelativePose().getRotation().getDegrees() ;
     double x2 = (Math.cos(Math.toRadians(angle)) * dist) - Units.inchesToMeters(5);
     double y2 = (Math.sin(Math.toRadians(angle)) * dist) + Units.inchesToMeters(10);
     double x = 0;
     double y = 0;
     if(camera_left.hasValidTarget() && camera_right.hasValidTarget())
     {
       x = (x2 + x1)/2;
       y = (y2 + y1)/2;
     }
     else if(!camera_left.hasValidTarget() && camera_right.hasValidTarget())
     {
       x = (x2);
       y = (y2);
     }
     else if(camera_left.hasValidTarget() && !camera_right.hasValidTarget())
     {
       x = (x1);
       y = (y1);
     }

     Pose2d pose = new Pose2d(-x,y,base.getLocalization().getRelativePose().getRotation());
     if(camera_right.hasValidTarget())
     {
      Pose2d pole = ReefPole.getPoleFromID(camera_right.getAprilTagID(), camera_right).getPose2d();
      base.getLocalization().getField2dObject("flipped").setPose(new Pose2d(pose.getY()+pole.getX(), pose.getX()+pole.getY(), pose.getRotation().plus(pole.getRotation())));
     base.getLocalization().getField2dObject("RobotPose").setPose(pose.relativeTo(ReefPole.getPoleFromID(camera_right.getAprilTagID(), camera_right).getPose2d()));
     base.getLocalization().getField2dObject("Tag").setPose(ReefPole.getPoleFromID(camera_right.getAprilTagID(), camera_right).getPose2d());
     }
     else if(camera_right.hasValidTarget())
     {
      Pose2d pole = ReefPole.getPoleFromID(camera_right.getAprilTagID(), camera_right).getPose2d();
      base.getLocalization().getField2dObject("flipped").setPose(new Pose2d(pose.getY()+pole.getX(), pose.getX()+pole.getY(), pose.getRotation().plus(pole.getRotation())));    
      base.getLocalization().getField2dObject("RobotPose").setPose(pose.relativeTo(ReefPole.getPoleFromID(camera_right.getAprilTagID(), camera_right).getPose2d()));
     base.getLocalization().getField2dObject("Tag").setPose(ReefPole.getPoleFromID(camera_right.getAprilTagID(), camera_right).getPose2d());
     }
     return pose;
   }

   public boolean closeEnough(String table)
    {
      LimeLight camera = base.getVision().getLimelight(table);
      return camera.hasValidTarget() && Math.abs(camera.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9]) <= 0.525 && Math.abs(camera.getTargetHorizontalOffset()) < 11;
    }

 
   // Called every time the scheduler runs while the command is scheduled.
   @Override
   public void execute() 
   {

    if(camera_left.hasValidTarget() || camera_right.hasValidTarget())
    {

      //ROTATION MEASUREMENT
      thetaMeasurement = camera_left.hasValidTarget() ? camera_left.getPoseEstimate(PoseEstimateType.TARGET_POSE_ROBOT_SPACE).getRaw()[4] : camera_right.getPoseEstimate(PoseEstimateType.TARGET_POSE_ROBOT_SPACE).getRaw()[4];
      thetaMeasurement = -filter.calculate(thetaMeasurement);

      //SETTING TAG ID
      if(tagId == -1)
      {
        if(camera_left.hasValidTarget())
        {
          tagId = (int)camera_left.getAprilTagID();
        }
        if(camera_right.hasValidTarget())
        {
          tagId = (int)camera_right.getAprilTagID();
        }
      }

      //CALCULATING FINAL DESIRED POSITION 
      if(camera_left.hasValidTarget() && (int)camera_left.getAprilTagID() == tagId)
      {
        // if(tagId != 19 && tagId != 20)
        // {
        goalPose2d = new Pose2d(Units.inchesToMeters(40), rightPole ? Units.inchesToMeters(0) : Units.inchesToMeters(0),new Rotation2d()).rotateAround(new Translation2d(0, 0), ReefPole.getPoleFromID(camera_left.getAprilTagID(), camera_left).getRotation());
        finalPose2d = new Pose2d(Units.inchesToMeters(15.5), rightPole ? Units.inchesToMeters(6.2) : Units.inchesToMeters(-11.5),new Rotation2d()).rotateAround(new Translation2d(0, 0), ReefPole.getPoleFromID(camera_left.getAprilTagID(), camera_left).getRotation());
        // }
        // else
        // {
        // goalPose2d = new Pose2d(Units.inchesToMeters(40), rightPole ? Units.inchesToMeters(0) : Units.inchesToMeters(0),new Rotation2d()).rotateAround(new Translation2d(0, 0), ReefPole.getPoleFromID(camera_left.getAprilTagID(), camera_left).getRotation());
        // finalPose2d = ReefPole.getPoleFromID(tagId, camera_left).getScoringPos();
        // }
      }

      if(camera_right.hasValidTarget() && (int)camera_right.getAprilTagID() == tagId)
      {
        // if(tagId != 19 && tagId != 20)
        // {
        goalPose2d = new Pose2d(Units.inchesToMeters(40), rightPole ? Units.inchesToMeters(0) : Units.inchesToMeters(0),new Rotation2d()).rotateAround(new Translation2d(0, 0), ReefPole.getPoleFromID(camera_right.getAprilTagID(), camera_right).getRotation());
        finalPose2d = new Pose2d(Units.inchesToMeters(15.5), rightPole ? Units.inchesToMeters(6.2) : Units.inchesToMeters(-11.5),new Rotation2d()).rotateAround(new Translation2d(0, 0), ReefPole.getPoleFromID(camera_right.getAprilTagID(), camera_right).getRotation());
        // }
        // else
        // {
        // goalPose2d = new Pose2d(Units.inchesToMeters(40), rightPole ? Units.inchesToMeters(0) : Units.inchesToMeters(0),new Rotation2d()).rotateAround(new Translation2d(0, 0), ReefPole.getPoleFromID(camera_right.getAprilTagID(), camera_right).getRotation());
        // finalPose2d = ReefPole.getPoleFromID(tagId, camera_right).getScoringPos();
        // }
      }
      
      if(camera_left.getAprilTagID() == tagId && camera_right.getAprilTagID() == tagId){
        base.getLocalization().resetRelativePose(getPose2d());
      }

      targetMeasurement = base.getLocalization().getRelativePose().getRotation().getDegrees() - thetaMeasurement;

    }

    //END COMMAND
    if(closeEnough("limelight-left") || closeEnough("limelight-right"))
    {
      isDone = true;
    }

    if(tagId != -1)
    {

    //PRIORITIZE FINAL POSITION OVER TRANSITION POSITION
    if(base.getLocalization().getRelativePose().getTranslation().getDistance(goalPose2d.getTranslation()) > base.getLocalization().getRelativePose().getTranslation().getDistance(finalPose2d.getTranslation()))
    {
      reached = true;
    }

  
    if(base.getLocalization().getRelativePose().getTranslation().getDistance(goalPose2d.getTranslation()) > 0.075 && !reached)
    {  
    //UP SPEED
    controller.getXController().setP(2);
    controller.getYController().setP(2);

    //CALCULATE SPEEDS
    double rSpeed = rController.calculate(base.getLocalization().getRelativePose().getRotation().getDegrees(), ReefPole.getPoleFromID(tagId, camera_left).getRotation().getDegrees() - 180);
    double xSpeed = controller.getXController().calculate(base.getLocalization().getRelativePose().getX(), goalPose2d.getX());
    double ySpeed = controller.getYController().calculate(base.getLocalization().getRelativePose().getY(), goalPose2d.getY());

    SmartDashboard.putNumber("Scoring Pos X", finalPose2d.getX());
    SmartDashboard.putNumber("Scoring Pos Y", finalPose2d.getY());
    SmartDashboard.putNumber("Tag ID", tagId);
    
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
    base.getDrivetrain().getRobotSpeeds().setSpeeds("feedback", spds);
    }
    else
    {
    reached = true;

    controller.getXController().setP(1.75);
    controller.getYController().setP(1.75);

    double rSpeed = rController.calculate(base.getLocalization().getRelativePose().getRotation().getDegrees(), ReefPole.getPoleFromID(tagId, camera_left).getRotation().getDegrees() - 180);
    double xSpeed = controller.getXController().calculate(base.getLocalization().getRelativePose().getX(), finalPose2d.getX());
    double ySpeed = controller.getYController().calculate(base.getLocalization().getRelativePose().getY(), finalPose2d.getY());

    //PUSH DATA
    SmartDashboard.putData("Rotation Controller", rController);
    SmartDashboard.putData("X Controller", controller.getXController());
    SmartDashboard.putData("Y Controller", controller.getYController());
    
    
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
    base.getDrivetrain().getRobotSpeeds().setSpeeds("feedback", spds);
  }
  }

    //SUPERSTRUCTURE LOGIC-------------------------------------------------------****

    
    double distance = 9999999;
    if(tagId != -1)
    {
     distance = Math.abs(base.getLocalization().getRelativePose().getTranslation().getDistance(finalPose2d.getTranslation()));
    }
    
    if(!superstructure.getStateMachine().getGoalState().equals(state.get()))
    {
    if(state.get().equals(SuperstructureState.L4))
    {
      if(distance < 2 && distance > 1)
      {
        superstructure.setSuper(SuperstructureState.L2);
      }
      else if(distance < 1 && distance > 0.5)
      {
        superstructure.setSuper(SuperstructureState.L3);
      }
      else if(distance < 0.3)
      {
        superstructure.setSuper(SuperstructureState.L4);
        
      }
       
    }
    else if(state.get().equals(SuperstructureState.L3))
    {
      if(distance < 2 && distance > 1)
      {
        superstructure.setSuper(SuperstructureState.L2);
      }
      else if(distance < 1)
      {
        superstructure.setSuper(SuperstructureState.L3);
        
      }  
    }
    else if(state.get().equals(SuperstructureState.L2))
    {
      if(distance < 1)
      {
        superstructure.setSuper(state.get());
       
      }
    }
    else if(state.get().equals(SuperstructureState.L1) && isDone)
    {
      superstructure.setSuper(SuperstructureState.L1);
      
    }
  }
  }

    //SUPERSTRUCTURE LOGIC-------------------------------------------------------****
 
 
   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {  
     base.getDrivetrain().getRobotSpeeds().setSpeeds("feedback", new ChassisSpeeds(0,0,0));
   }
 
   // Returns true when the command should end.
   @Override
   public boolean isFinished() 
   {  
     return isDone;
   }
 }