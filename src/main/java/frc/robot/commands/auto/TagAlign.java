// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import ca.frc6390.athena.controllers.DelayedOutput;
import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.core.RobotSpeeds.SpeedSource;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateType;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatencyType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

public class TagAlign extends Command {
  public RobotBase<?> base;
  public double AutoDistToTrigger;
  public double DistToCommand;
  public Command command;
  public LimeLight ll;
  public int runTag;
  public Pose2d curPose;
  public MedianFilter filter;
  public double thetaMeasurement;
  public ProfiledPIDController xController = new ProfiledPIDController(2, 0, 0, new Constraints(1, 1));
  public ProfiledPIDController yController =  new ProfiledPIDController(4, 0.1, 0, new Constraints(2, 1));
  public PIDController rController = new PIDController(0.03, 0, 0);
  public DelayedOutput endCommand;
  public DelayedOutput noTag;
  
  public TagAlign(RobotBase<?> base,String lltable, Command command, double DistToCommand)
  {
   this.base = base;
   this.command = command;
   this.DistToCommand = DistToCommand;
   this.ll = this.base.getVision().getLimelight(lltable);
  }

  public TagAlign(RobotBase<?> base, String lltable)
  {
   this(base, lltable, Commands.none(), 0);
  }

  @Override
  public void initialize() 
  {
    runTag = -1;
    endCommand = new DelayedOutput(() -> closeEnough(), 0.75);
    curPose = new Pose2d();
    filter = new MedianFilter(25);
    noTag = new DelayedOutput(() -> hasNoTag(), 1);
    thetaMeasurement = 0;
    rController.setIntegratorRange(-5, 5);
  }

  public boolean hasNoTag()
  {
    return !(runTag == (int)ll.getAprilTagID()) || !ll.hasValidTarget();
  }
  public Pose2d getBotPoseTagSpace(LimeLight ll)
  {
    double dist = ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9];
    double angle = ll.getTargetHorizontalOffset();
    double x = (Math.cos(Math.toRadians(angle)) * dist);
    double y = (Math.sin(Math.toRadians(angle)) * dist); 
    SmartDashboard.putNumber("x", ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_BLUE).getLocalizationPose().getX());
    SmartDashboard.putNumber("Y", ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_BLUE).getLocalizationPose().getY());
    SmartDashboard.putNumber("Angle", thetaMeasurement);
    

    return new Pose2d(x,y, base.getLocalization().getFieldPose().getRotation());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
       
    SmartDashboard.putBoolean("ENd ",endCommand.getAsBoolean());
SmartDashboard.putNumber("Dist", ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9]);
    if(ll.hasValidTarget())
    { 
      if(runTag == -1)
      {
        runTag = (int)ll.getAprilTagID();
      }
      curPose = getBotPoseTagSpace(ll);
      thetaMeasurement =-filter.calculate(ll.getPoseEstimate(PoseEstimateType.TARGET_POSE_ROBOT_SPACE).getRaw()[4]);
    }
    else
    {
      thetaMeasurement = 0;
    }

    // if(ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9] > 1.5)
    // {
    //   xController.setP(2);
    // }
    // else
    // {
    //   xController.setP(1);
    // }

   

    if(ll.hasValidTarget())
    {

      double Xspeed = -xController.calculate(curPose.getX(), Units.inchesToMeters(17.5));
      double YSpeed = yController.calculate(curPose.getY(),0);
      double rSpeed = rController.calculate(thetaMeasurement, 0);//* Math.copySign(1, ll.getTargetHorizontalOffset()), 0);
   
      
    // if(DriverStation.isAutonomous())
    // {
      if(runTag == (int)ll.getAprilTagID())
      {
        if(ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9] <= DistToCommand)
        {
          CommandScheduler.getInstance().schedule(command);
        }
        SmartDashboard.putBoolean("ALGIN TAKEN",true);
        base.getDrivetrain().getRobotSpeeds().enableSpeeds(SpeedSource.AUTO, false);
        // base.getDrivetrain().getRobotSpeeds().stopAutoSpeeds();
        base.getDrivetrain().getRobotSpeeds().setFeedbackSpeeds(Xspeed, YSpeed, -rSpeed);
      }
      else
      {
        SmartDashboard.putBoolean("ALGIN TAKEN",false);
        base.getDrivetrain().getRobotSpeeds().enableSpeeds(SpeedSource.AUTO, true);
        base.getDrivetrain().getRobotSpeeds().stopFeedbackSpeeds();
      }
    }
    else
    {
      SmartDashboard.putBoolean("ALGIN TAKEN",false);
      
      base.getDrivetrain().getRobotSpeeds().enableSpeeds(SpeedSource.AUTO, true);
      // if(base.getDrivetrain().getRobotSpeeds().getAutoSpeeds().equals(new ChassisSpeeds(0,0,0)))
      // {
      //   base.getDrivetrain().getRobotSpeeds().setFeedbackSpeeds(-1, 0,0);
      // }
      // else{
      // base.getDrivetrain().getRobotSpeeds().stopFeedbackSpeeds();
      // }
    }
    
  }

  public boolean closeEnough()
  {
    return ll.hasValidTarget() && ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9] <= 0.5 && ll.getTargetHorizontalOffset() < 3.5;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    if(ll.hasValidTarget())
    {
    base.getLocalization().resetFieldPose(ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_BLUE).getLocalizationPose().getX(), ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_BLUE).getLocalizationPose().getY());
    }
    base.getDrivetrain().getRobotSpeeds().enableSpeeds(SpeedSource.AUTO, true);
    base.getDrivetrain().getRobotSpeeds().setFeedbackSpeeds(0,0,0);
    base.getDrivetrain().getRobotSpeeds().stopFeedbackSpeeds();
    base.getDrivetrain().getRobotSpeeds().setAutoSpeeds(0,0,0);
    base.getDrivetrain().getRobotSpeeds().stopAutoSpeeds();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if(DriverStation.isAutonomous())
    {
      return endCommand.getAsBoolean() || noTag.getAsBoolean();
    }
    else
    {
    return noTag.getAsBoolean();
    }
    
  }
}
