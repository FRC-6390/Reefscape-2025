// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotation;

import javax.print.attribute.standard.Media;
import javax.sound.sampled.Port;

import ca.frc6390.athena.controllers.DebouncedController;
import ca.frc6390.athena.core.RobotLocalization;
import ca.frc6390.athena.core.RobotVision;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.filters.FilterList;
import ca.frc6390.athena.filters.FilteredValue;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.AutoAlignHelper;
public class AlignTets extends Command {
  
  public LimeLight limelight; 
  public SwerveDrivetrain drivetrain;
  public DebouncedController cont; 
  public boolean closeEnough;
  public boolean isDone;
  public ChassisSpeeds speeds;
  public int runTag;
  public boolean hasSet;
  public RobotLocalization localization;
  public ALIGNMODE mode;
  public AutoAlignHelper info;

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
  
  public AlignTets(LimeLight limeLight, SwerveDrivetrain drivetrain, DebouncedController cont, ALIGNMODE mode, RobotLocalization localization) {
    this.drivetrain = drivetrain; 
    this.cont = cont;
    this.localization = localization;
    limelight = limeLight;
    this.mode = mode;
  }

  public AlignTets(LimeLight limeLight, SwerveDrivetrain drivetrain, DebouncedController cont, ALIGNMODE mode, RobotLocalization localization, int tagNum) {
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
    info = new AutoAlignHelper(limelight, localization, drivetrain);
  }


  @Override
  public void execute() 
  {
    if(limelight.hasValidTarget()){
      if(!hasSet){
        hasSet = true;
        runTag = ((int)limelight.getAprilTagID());
      }
      if(((int)limelight.getAprilTagID()) == runTag) {
      info.gatherData();
      speeds = info.calculateSpeeds(mode, true);
      if(limelight.getTargetArea() > 20){
        closeEnough = true;
      }
      }
      else{
        speeds = info.calculateSpeeds(mode, false);
      }
    }
    else{
      speeds = info.calculateSpeeds(mode, false);
    }
    if(hasSet) {
      // if(closeEnough  && distanceSensor.isRangeValid() && distanceSensor.getRange(Unit.kInches) < 16){
      //   isDone = true;
      // } 
      drivetrain.feedbackSpeeds(speeds);
    }
  }

  @Override
  public void end(boolean interrupted) 
  {
    drivetrain.drive(new ChassisSpeeds(0,0,0));
    drivetrain.feedbackSpeeds(new ChassisSpeeds(0,0,0));
    SmartDashboard.putNumber("Is Running", 0);
  }

  @Override
  public boolean isFinished() {
    return isDone;
  }

}
