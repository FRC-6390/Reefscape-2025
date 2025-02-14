// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import au.grapplerobotics.LaserCan;
import ca.frc6390.athena.controllers.EnhancedXboxController;
import ca.frc6390.athena.core.RobotLocalization;
import ca.frc6390.athena.core.RobotSpeeds.SpeedAxis;
import ca.frc6390.athena.core.RobotSpeeds.SpeedSource;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.AutoAlignHelper;
public class AutoAlign extends Command {
  
  public LimeLight limelight; 
  public SwerveDrivetrain drivetrain;
  public EnhancedXboxController cont; 
  public boolean closeEnough;
  public boolean isDone;
  public ChassisSpeeds speeds;
  public int runTag;
  public boolean hasSet;
  public RobotLocalization localization;
  public ALIGNMODE mode;
  public AutoAlignHelper helper;
  private LaserCan las = new LaserCan(59);

  public enum ALIGNMODE
  {
    FEEDER(1),
    REEF(-1);

    double num;
    private ALIGNMODE(double num)
    {
      this.num = num;
    } 

    public double get()
    {
      return num;
    }
  }
  
  public AutoAlign(LimeLight limeLight, SwerveDrivetrain drivetrain, EnhancedXboxController cont, ALIGNMODE mode, RobotLocalization localization) {
    this.drivetrain = drivetrain; 
    this.cont = cont;
    this.localization = localization;
    limelight = limeLight;
    this.mode = mode;
  }

  public AutoAlign(LimeLight limeLight, SwerveDrivetrain drivetrain, EnhancedXboxController cont, ALIGNMODE mode, RobotLocalization localization, int tagNum) {
    this.drivetrain = drivetrain; 
    this.cont = cont;
    limelight = limeLight;
    this.mode = mode;
    runTag = tagNum;
    if(runTag == tagNum)
    {
    hasSet = true;
    }
  }

  @Override
  public void initialize() 
  {
    closeEnough = false;
    hasSet = false;
    isDone =false;
    speeds = new ChassisSpeeds();
    runTag = -1;
    helper = new AutoAlignHelper(limelight, localization, drivetrain);
  }


  @Override
  public void execute() 
  {
    System.out.println("Executing");
    if(limelight.hasValidTarget()){
      
      if(!hasSet){
        drivetrain.getRobotSpeeds().enableSpeeds(SpeedSource.AUTO, false);
        hasSet = true;
        runTag = ((int)limelight.getAprilTagID());
      }
      if(((int)limelight.getAprilTagID()) == runTag) {
      
      helper.gatherData();
      speeds = helper.calculateSpeeds(mode, true);
      if(limelight.getTargetArea() > 10){
        closeEnough = true;
      }
      }
      else{
        speeds = helper.calculateSpeeds(mode, false);
      }
    }
    else{
      speeds = helper.calculateSpeeds(mode, false);
    }
    if(hasSet) {
      if(closeEnough  && las.getMeasurement().status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && las.getMeasurement().distance_mm < 100){
        isDone = true;
      } 
      SmartDashboard.putNumber("Dist", las.getMeasurement().distance_mm);
      drivetrain.getRobotSpeeds().setFeedbackSpeeds(speeds);
    }
  }

  @Override
  public void end(boolean interrupted) 
  {
    drivetrain.getRobotSpeeds().enableSpeeds(SpeedSource.AUTO, true);
    drivetrain.getRobotSpeeds().setFeedbackSpeeds(new ChassisSpeeds(0,0,0));
    System.out.println("Done!");
  }

  @Override
  public boolean isFinished() {
    return isDone;
  }

}
