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
import frc.robot.subsystems.superstructure.CANdleSubsystem;
import frc.robot.utils.ReefScoringPos.ReefPole;

public class TagAlign extends Command {
  public RobotBase<?> base;
  public double AutoDistToTrigger;
  public static double horizonalTolerance = 2.5;
  public double DistToCommand;
  public Command command;
  public static LimeLight ll;
  public int runTag;
  public Pose2d curPose;
  public MedianFilter filter;
  public double thetaMeasurement;
  public ProfiledPIDController xController = new ProfiledPIDController(2, 0, 0, new Constraints(0.25, 0.25));
  public ProfiledPIDController yController =  new ProfiledPIDController(4, 0.1, 0, new Constraints(1, 1));
  public PIDController rController = new PIDController(0.04, 0, 0);
  public DelayedOutput endCommand;
  public DelayedOutput noTag;
  public ReefPole pole;
  public CANdleSubsystem candle;
  

  public TagAlign(RobotBase<?> base,String lltable, Command command, double DistToCommand, ReefPole pole, CANdleSubsystem candle)
  {
   this.base = base;
   this.pole = pole;
   this.command = command;
   this.candle = candle;
   this.DistToCommand = DistToCommand;
   ll = this.base.getVision().getLimelight(lltable);
  }

  public TagAlign(RobotBase<?> base, String lltable, CANdleSubsystem candle)
  {
   this(base, lltable, Commands.none(), Double.POSITIVE_INFINITY, ReefPole.NONE, candle);
  }

  public TagAlign(RobotBase<?> base, String lltable, Command command, double DistToCommand, CANdleSubsystem caNdleSubsystem)
  {
   this(base, lltable, command, DistToCommand, ReefPole.NONE, caNdleSubsystem);
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
    ll.setPriorityID((int)pole.getApriltagId());
    xController.setIntegratorRange(-0.05, 0.05);
    xController.reset(curPose.getX());
    yController.reset(curPose.getY());

    // yController.setIntegratorRange(-0.2, 0.2);
  }

  public boolean hasNoTag()
  {
    return !(runTag == (int)ll.getAprilTagID()) || !ll.hasValidTarget();
  }
  public Pose2d getBotPoseTagSpace(LimeLight ll)
  {
    double dist = ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9];
    double offset = 0;
    if(!pole.equals(ReefPole.NONE))
    {
      offset = pole.getOffsetInDegrees();
    }

    double angle = ll.getTargetHorizontalOffset();
    double x = (Math.cos(Math.toRadians(angle)) * dist);
    double y = (Math.sin(Math.toRadians(angle)) * dist); 
    SmartDashboard.putNumber("x",x);
    SmartDashboard.putNumber("Y", y);
    SmartDashboard.putNumber("Angle", angle);
    

    return new Pose2d(x,y, base.getLocalization().getFieldPose().getRotation());
  }

  @Override
  public void execute() 
  {
    candle.setRGB(255, 0, 0);
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

    double Xspeed =0; //-xController.calculate(curPose.getX(), Units.inchesToMeters(18));
    double YSpeed =0;// yController.calculate(curPose.getY(),0);
    double rSpeed =0;// rController.calculate(thetaMeasurement, 0); 


  
    if(pole.getApriltagId() != -1)
    {

    if(runTag == pole.getApriltagId())
    {
      xController.calculate(curPose.getX(), Units.inchesToMeters(18));
      yController.calculate(curPose.getY(),0);
      rController.calculate(thetaMeasurement, 0);
      
     Xspeed =  -xController.getSetpoint().velocity;
     YSpeed =  yController.getSetpoint().velocity;
     rSpeed =  rController.getSetpoint();
    }
    else
    {
      Xspeed = 0;
      YSpeed = 0;
      rSpeed = 0;
    }
  }

  // if(ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9] > 1)
  // {
  //   yController.setP(2);
  // }
  // else
  // {
  //   yController.setP(1);
  // }

  
    if(ll.hasValidTarget())
    {
      if(runTag == (int)ll.getAprilTagID())
      {
  
        if(ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9] <= DistToCommand)
        {
          // CommandScheduler.getInstance().schedule(command);
        }
        SmartDashboard.putNumber("XSpeed", Xspeed);
        SmartDashboard.putNumber("YSpeed", YSpeed);

        SmartDashboard.putNumber("RSpeed", rSpeed);

        SmartDashboard.putBoolean("ALGIN TAKEN",true);
        base.getDrivetrain().getRobotSpeeds().setSpeedSourceState("auto", false);
        candle.setRGB(0, 255, 0);
        base.getDrivetrain().getRobotSpeeds().setSpeeds("feedback", Xspeed, YSpeed, -rSpeed);
      }
      else
      {
        candle.setRGB(255, 0, 0);

        SmartDashboard.putBoolean("ALGIN TAKEN",false);
        base.getDrivetrain().getRobotSpeeds().setSpeedSourceState("auto", false);
        base.getDrivetrain().getRobotSpeeds().stopSpeeds("feedback");
      }
  }
    else
    {
      candle.setRGB(255, 0, 0);

      SmartDashboard.putBoolean("ALGIN TAKEN",false);
      base.getDrivetrain().getRobotSpeeds().setSpeedSourceState("auto", false);
      base.getDrivetrain().getRobotSpeeds().setSpeeds("feedback", -0.2, YSpeed, rSpeed);
      
    }
  }

  public boolean closeEnough()
  {
    double offset = 0;
    if(!pole.equals(ReefPole.NONE))
    {
      offset = pole.getOffsetInDegrees();
    }
    return ll.hasValidTarget() && ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9] <= 0.5 && Math.abs(Math.abs(ll.getTargetHorizontalOffset()) - Math.abs(offset)) < horizonalTolerance;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    candle.setRGB(0, 0, 0);

    if(ll.hasValidTarget())
    {
    base.getLocalization().resetFieldPose(ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_BLUE).getLocalizationPose().getX(), ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_BLUE).getLocalizationPose().getY());
    }
    base.getDrivetrain().getRobotSpeeds().setSpeedSourceState("auto", true);
    
    base.getDrivetrain().getRobotSpeeds().stopSpeeds("feedback");
    base.getDrivetrain().getRobotSpeeds().stopSpeeds("auto");

  }

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
