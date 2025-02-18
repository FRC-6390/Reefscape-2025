// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import au.grapplerobotics.LaserCan;
import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.core.RobotDrivetrain;
import ca.frc6390.athena.core.RobotLocalization;
import ca.frc6390.athena.core.RobotSpeeds.SpeedSource;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.AutoAlignHelper;
import frc.robot.utils.ReefScoringPos.ReefPole;
public class AutoAlign extends Command {
  
  public LimeLight limelight; 
  public RobotDrivetrain<?> drivetrain;
  public boolean closeEnough;
  public boolean isDone;
  public ChassisSpeeds speeds;
  public long runTag;
  public long tagNum = -1;
  public boolean hasSet;
  public RobotLocalization localization;
  public static boolean idling = false;
  public AutoAlignHelper helper;
  public Command event;
  private LaserCan las = new LaserCan(59);
  public double distToTrigger;

  public AutoAlign(RobotBase<?> base, ReefPole pole) {
    this(base.getCameraFacing(pole.getTranslation()), base, pole.getApriltagId());
  }
  
  public AutoAlign(String limelight, RobotBase<?> base) {
    this(limelight, base, -1);
  }

  public AutoAlign(String limelight, RobotBase<?> base, long tagNum) {
    this(base.getVision().getCamera(limelight), base, -1);
  }

  public AutoAlign(LimeLight limelight, RobotBase<?> base, long tagNum) {
    this.drivetrain = base.getDrivetrain(); 
    this.limelight = limelight;
    this.localization = base.getLocalization();
    this.tagNum = tagNum;
  }

  @Override
  public void initialize() 
  {
    closeEnough = false;
    hasSet = false;
    isDone =false;
    speeds = new ChassisSpeeds();
    runTag = tagNum;
    helper = new AutoAlignHelper(limelight, localization, drivetrain);
    helper.reset();
  }


  @Override
  public void execute() 
  {
    if(!idling)
    {
    SmartDashboard.putBoolean("Distance Valid", las.getMeasurement().status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);
    SmartDashboard.putBoolean("Close Enough", closeEnough);
    SmartDashboard.putNumber("Run Tag", runTag);
    SmartDashboard.putBoolean("Has Set", hasSet);
    if(limelight.hasValidTarget()){
      
      if(!hasSet){
        drivetrain.getRobotSpeeds().enableSpeeds(SpeedSource.AUTO, false);
        hasSet = true;
        if(runTag == -1)
        {
        runTag = limelight.getAprilTagID();
        }
        drivetrain.getRobotSpeeds().enableSpeeds(SpeedSource.AUTO, false);
      }
      if(limelight.getAprilTagID() == runTag) {
      drivetrain.getRobotSpeeds().enableSpeeds(SpeedSource.AUTO, false);
      helper.gatherData();
      speeds = helper.calculateSpeeds(limelight,true);
      if(limelight.getTargetArea() > 10){
        closeEnough = true;
      }
      if(las.getMeasurement().status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && las.getMeasurement().distance_mm < 500)
      {
        helper.setP(0.025);
      }
      }
      else{
        // drivetrain.getRobotSpeeds().enableSpeeds(SpeedSource.AUTO, true);
        // speeds = helper.calculateSpeeds(mode, false);
        speeds = new ChassisSpeeds(0, 0,0);
      }
    }
    else{
      // drivetrain.getRobotSpeeds().enableSpeeds(SpeedSource.AUTO, true);
      // speeds = helper.calculateSpeeds(mode, false);
      // speeds = new ChassisSpeeds();
      speeds = new ChassisSpeeds(-limelight.config.getAngleCos(), 0,0);
    }
    if(hasSet) {
      if(las.getMeasurement() != null)
      {
      if(closeEnough  && las.getMeasurement().status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && las.getMeasurement().distance_mm < 120){
        isDone = true;
      } 
      }
      else
      {
        if(closeEnough){
          isDone = true;
        } 
      }

      if(las.getMeasurement().status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)
      {
        if(las.getMeasurement().distance_mm < distToTrigger)
        {
        CommandScheduler.getInstance().schedule(event);
        }
      }
      drivetrain.getRobotSpeeds().setFeedbackSpeeds(speeds);
    }
    else{
      drivetrain.getRobotSpeeds().setFeedbackSpeeds(new ChassisSpeeds(-limelight.config.getAngleCos() / 2,0,0));
    }
  }
    else
    {
      System.out.println("Idling");
    }
}

  @Override
  public void end(boolean interrupted) 
  {
    drivetrain.getRobotSpeeds().enableSpeeds(SpeedSource.AUTO, true);
    drivetrain.getRobotSpeeds().stopFeedbackSpeeds();
    System.out.println("Done!");
  }

  @Override
  public boolean isFinished() {
    return isDone;
  }

}
