// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotation;

import java.util.function.DoubleSupplier;

import javax.print.attribute.standard.Media;
import javax.sound.sampled.Port;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import au.grapplerobotics.LaserCan;
import ca.frc6390.athena.controllers.DebouncedController;
import ca.frc6390.athena.core.RobotLocalization;
import ca.frc6390.athena.core.RobotVision;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain.AXIS;
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
    distanceSensor.setEnabled(true);
    distanceSensor.setAutomaticMode(true);
    info = new AutoAlignHelper(limelight, localization, drivetrain);
  }


  @Override
  public void execute() 
  {
    System.out.println("Executing");
    
    SmartDashboard.putNumber("X", drivetrain.getDriveSpeeds().vxMetersPerSecond);
    SmartDashboard.putNumber("Y", drivetrain.getDriveSpeeds().vyMetersPerSecond);
    SmartDashboard.putNumber("Rot", drivetrain.getDriveSpeeds().omegaRadiansPerSecond);
    if(limelight.hasValidTarget()){
      
      if(!hasSet){
        hasSet = true;
        runTag = ((int)limelight.getAprilTagID());
        drivetrain.disableAxis(AXIS.Disable);
      }
      if(((int)limelight.getAprilTagID()) == runTag) {
      helper.gatherData();
      speeds = helper.calculateSpeeds(mode, true);
      if(limelight.getTargetArea() > 12){
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
      if(closeEnough  && distanceSensor.isRangeValid() && distanceSensor.getRange(Unit.kInches) < 16){
        isDone = true;
      } 
      drivetrain.feedbackSpeeds(speeds);
    }
  }

  @Override
  public void end(boolean interrupted) 
  {
    drivetrain.drive(new ChassisSpeeds(0,0,0));
    drivetrain.feedbackSpeeds(new ChassisSpeeds(0,0,0));
    drivetrain.disableAxis(AXIS.Enable);
    System.out.println("Done!");
  }

  @Override
  public boolean isFinished() {
    return isDone;
  }

}
