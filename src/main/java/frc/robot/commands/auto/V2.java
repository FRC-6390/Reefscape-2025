

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
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
  public boolean rightPole = false;
  public double thetaMeasurement = 0;
  
  public Superstructure superstructure;
  public Supplier<SuperstructureState> state;
  public int tagId = -1;
  public MedianFilter filter;
  public Trajectory trajectory; 
  public double targetMeasurement;
  public double startTime;
  public List<SuperstructureState> intermdiateStates;
  public Pose2d finalPose2d = new Pose2d(Units.inchesToMeters(5), Units.inchesToMeters(6.5),new Rotation2d());
  public PIDController rController = new PIDController(0.04, 0, 0);
  //0.04

  public HolonomicDriveController controller = new HolonomicDriveController(
                                                          new PIDController(1, 0, 0), 
                                                          new PIDController(1, 0, 0),
                                                          new ProfiledPIDController(0, 0, 0, new Constraints(0, 0)));
  public boolean isDone = false;



   public V2(RobotBase<?> base,String lltable, boolean rightPole, Superstructure superstructure, Supplier<SuperstructureState> state)
   {
    lr = base.getVision().getLimelight("limelight-right");
    ll = base.getVision().getLimelight("limelight-left");
    this.superstructure = superstructure;
    this.state = state;
    this.base = base;
    robotSpeeds = base.getRobotSpeeds();
    this.rightPole = rightPole; 
    tagId = -1;
    isDone = false;
   }

   @Override
   public void initialize() 
   {
    thetaMeasurement = 0;
    // rController.enableContinuousInput(-180, 180);
    rController.setTolerance(5);

    isDone = false;
    filter = new MedianFilter(50);
    tagId = -1;

    reached = false;
    if(ll.hasValidTarget())
    {
      base.getLocalization()
      .resetRelativePose
      (
        getPose2d()
      );
      tagId = ((int)ll.getAprilTagID());
    } 
    else if(lr.hasValidTarget())
    {
      base.getLocalization()
      .resetRelativePose
      (
        getPose2d()
      );
      tagId = ((int)lr.getAprilTagID());
    } 

    // trajectory = TrajectoryGenerator.generateTrajectory(List.of(base.getLocalization().getRelativePose(), new Pose2d(0, 0, new Rotation2d())), new TrajectoryConfig(4, 4));

  }
 

  
  
   public Pose2d getPose2d()
   {

    LimeLight ll1 = base.getVision().getLimelight("limelight-left");
    double dist1 = ll1.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9];
    double angle1 =  ll1.getTargetHorizontalOffset() -base.getLocalization().getRelativePose().getRotation().getDegrees() ;
    double x1 = (Math.cos(Math.toRadians(angle1)) * dist1) - Units.inchesToMeters(5);
    double y1 = (Math.sin(Math.toRadians(angle1)) * dist1)- Units.inchesToMeters(10);; 

    
     LimeLight ll = base.getVision().getLimelight("limelight-right");
     double dist = ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9];
     double angle =  ll.getTargetHorizontalOffset() -base.getLocalization().getRelativePose().getRotation().getDegrees() ;
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

     Pose2d pose = new Pose2d(-x,y,base.getLocalization().getRelativePose().getRotation());
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
      goalPose2d = new Pose2d(Units.inchesToMeters(30), rightPole ? Units.inchesToMeters(0) : Units.inchesToMeters(0),new Rotation2d()).rotateAround(new Translation2d(), ReefPole.getPoleFromID(ll.getAprilTagID(), ll).getRotation());
      finalPose2d = new Pose2d(Units.inchesToMeters(9), rightPole ? Units.inchesToMeters(10) : Units.inchesToMeters(-10),new Rotation2d()).rotateAround(new Translation2d(), ReefPole.getPoleFromID(ll.getAprilTagID(), ll).getRotation());
      }
      if(lr.hasValidTarget() && (int)lr.getAprilTagID() == tagId)
      {
      goalPose2d = new Pose2d(Units.inchesToMeters(30), rightPole ? Units.inchesToMeters(0) : Units.inchesToMeters(0),new Rotation2d()).rotateAround(new Translation2d(), ReefPole.getPoleFromID(lr.getAprilTagID(), lr).getRotation());
      finalPose2d = new Pose2d(Units.inchesToMeters(9), rightPole ? Units.inchesToMeters(10) : Units.inchesToMeters(-10),new Rotation2d()).rotateAround(new Translation2d(), ReefPole.getPoleFromID(lr.getAprilTagID(), lr).getRotation());
      }
    
      base.getLocalization()
      .resetRelativePose
      (
        getPose2d()
      );

      targetMeasurement = base.getLocalization().getRelativePose().getRotation().getDegrees() - thetaMeasurement;

    }

    if(closeEnough("limelight-left") || closeEnough("limelight-right"))
    {
      isDone = true;
      superstructure.setSuper(SuperstructureState.Score);
    }

    if(tagId != -1)
    {

    if(base.getLocalization().getRelativePose().getTranslation().getDistance(goalPose2d.getTranslation()) > 0.075 && !reached)
    {  
    controller.getXController().setP(2);
    controller.getYController().setP(2);

  
    double rSpeed = rController.calculate(base.getLocalization().getRelativePose().getRotation().getDegrees(), ReefPole.getPoleFromID(tagId, ll).getRotation(Alliance.Red).getDegrees() - 180);
    SmartDashboard.putNumber("R speed", rSpeed);
    SmartDashboard.putNumber("R Error", rController.getError());
    SmartDashboard.putNumber("R Error", base.getLocalization().getRelativePose().getRotation().getDegrees());
    SmartDashboard.putNumber("Pole", ReefPole.getPoleFromID(tagId, ll).getRotation(Alliance.Red).getDegrees() - 180);
    

    double xSpeed = controller.getXController().calculate(base.getLocalization().getRelativePose().getX(), goalPose2d.getX());
    double ySpeed = controller.getYController().calculate(base.getLocalization().getRelativePose().getY(), -goalPose2d.getY());
    
    
    SmartDashboard.putNumber("Y PJJKN", base.getLocalization().getRelativePose().getY());
    SmartDashboard.putNumber("Y Set", goalPose2d.getY());

    base.getDrivetrain().getRobotSpeeds().setSpeeds("feedback", new ChassisSpeeds(0, ySpeed, 0));
    // base.getDrivetrain().getRobotSpeeds().setSpeeds("feedback", new ChassisSpeeds(0, 0, rSpeed));

    }
    else
    {
    reached = true;
    double rSpeed = rController.calculate(base.getLocalization().getRelativePose().getRotation().getDegrees(), ReefPole.getPoleFromID(tagId, ll).getRotation(Alliance.Red).getDegrees() - 180);
    
    controller.getXController().setP(1.25);
    controller.getYController().setP(1.25);
    double xSpeed = controller.getXController().calculate(base.getLocalization().getRelativePose().getX(), finalPose2d.getX());
    double ySpeed = controller.getYController().calculate(base.getLocalization().getRelativePose().getY(), -finalPose2d.getY());
    
    SmartDashboard.putNumber("Y PJJKN", base.getLocalization().getRelativePose().getY());
    SmartDashboard.putNumber("Y Set", finalPose2d.getY());
    base.getDrivetrain().getRobotSpeeds().setSpeeds("feedback", new ChassisSpeeds(0, xSpeed, 0));
        // base.getDrivetrain().getRobotSpeeds().setSpeeds("feedback", new ChassisSpeeds(0, 0, rSpeed));

    }
  }

    //SUPERSTRUCTURE LOGIC-------------------------------------------------------****

    
    double distance = 9999999;
    if(tagId != -1)
    {
     distance = Math.abs(base.getLocalization().getRelativePose().getTranslation().getDistance(finalPose2d.getTranslation()));
    }
    
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

    //SUPERSTRUCTURE LOGIC-------------------------------------------------------****
 
 
   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {
     
     base.getDrivetrain().getRobotSpeeds().setSpeeds("feedback", new ChassisSpeeds(0,0,0));
    //  if(closeEnough("limelight-left") || closeEnough("limelight-right"))
    //  {
    //   superstructure.setSuper(SuperstructureState.Score);
    //  }
   }
 
   // Returns true when the command should end.
   @Override
   public boolean isFinished() {
     
     return isDone;
   }
 }