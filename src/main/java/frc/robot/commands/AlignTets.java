// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotation;

import javax.print.attribute.standard.Media;
import javax.sound.sampled.Port;

import com.revrobotics.Rev2mDistanceSensor.Unit;

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
import frc.robot.utils.AutoAlignInfo;
import frc.robot.utils.DistanceSensor;
public class AlignTets extends Command {
  public LimeLight limelight; 
  public SwerveDrivetrain drivetrain;
  public DebouncedController cont; 
  public PIDController controller = new PIDController(0.025, 0, 0);
  public PIDController xController = new PIDController(0.065, 0, 0.0001);
  public MedianFilter yFilter = new MedianFilter(10);
  public MedianFilter filter = new MedianFilter(10);
  public boolean closeEnough;
  public boolean isDone;
  public ChassisSpeeds speeds;
  public int runTag;
  public DistanceSensor distanceSensor = new DistanceSensor(com.revrobotics.Rev2mDistanceSensor.Port.kOnboard);
  public boolean hasSet;
  public Rotation2d lastYaw;
  public Rotation2d lastRobotYaw;
  public Pose2d lastRObotPose2d;
  public double xVelocity;
  public double xMeasurement;
  public double thetaMeasurement;
  public double yVelocity;
  public FilterList x;
  public FilterList y;
  public PIDController xController2 = new PIDController(1.2, 0,0);
  public Pose2d botPose;
  public double yMeasurement;
  public double rVel;
  public RobotLocalization localization;
  public ALIGNMODE mode;
  public AutoAlignInfo info;
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
    rVel = 0;
    xVelocity = 0;
    yVelocity = 0;
    lastYaw = new Rotation2d();
    lastRobotYaw  =new Rotation2d();
    thetaMeasurement =0;
    xMeasurement = 0;
    speeds =new ChassisSpeeds();
    runTag = -1;
    lastRObotPose2d = new Pose2d();
    distanceSensor.setEnabled(true);
    distanceSensor.setAutomaticMode(true);
    botPose = new Pose2d();
    x =  new FilterList().addMedianFilter(10);
    y = new FilterList().addMedianFilter(50);
    info = new AutoAlignInfo(limelight, localization, drivetrain);
 
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
      // thetaMeasurement = -filter.calculate(limelight.getPoseEstimate(PoseEstimateType.TARGET_POSE_ROBOT_SPACE).getRaw()[4]);
      // lastYaw = Rotation2d.fromDegrees(thetaMeasurement);
      // lastRobotYaw = Rotation2d.fromRadians(MathUtil.angleModulus(drivetrain.getIMU().getYaw().getRadians()));
      // lastRObotPose2d = localization.getRelativePose();
      // localization.resetRelativePose(0,  0, 0);
      // Pose2d pos = limelight.getPoseEstimate(PoseEstimateType.TARGET_POSE_ROBOT_SPACE).getPose();
      // botPose = new Pose2d(-x.calculate(pos.getX()), -y.calculate(pos.getY()), pos.getRotation());
      // xMeasurement = limelight.getTargetHorizontalOffset();
      rVel = -controller.calculate(info.getThetatMeasurement(), 0);
      if(limelight.getTargetArea() > 20){
        closeEnough = true;
      }
      }
      else{
        yVelocity = xController2.calculate(localization.getRelativePose().getX(),info.getTargetPoseRobotSpace().getX());
        xVelocity = xController2.calculate(localization.getRelativePose().getY(),info.getTargetPoseRobotSpace().getY());
        double rot = info.getLastRobotYaw().getDegrees() + info.getLastYaw().getDegrees();
        rVel = controller.calculate(MathUtil.angleModulus(drivetrain.getIMU().getYaw().getRadians()) * 180/Math.PI, rot);
      }
    }
    else{
      yVelocity = xController2.calculate(localization.getRelativePose().getX(),info.getTargetPoseRobotSpace().getX());
      xVelocity = xController2.calculate(localization.getRelativePose().getY(),info.getTargetPoseRobotSpace().getY());
      double rot = info.getLastRobotYaw().getDegrees() + info.getLastYaw().getDegrees();
      rVel = controller.calculate(MathUtil.angleModulus(drivetrain.getIMU().getYaw().getRadians()) * 180/Math.PI, rot);
    }
    if(hasSet) {
      if(limelight.hasValidTarget()){
      xVelocity = mode.get() * xController.calculate(info.getXMeasurement() , 0);
      yVelocity = mode.get();
      }
      if(closeEnough  && distanceSensor.isRangeValid() && distanceSensor.getRange(Unit.kInches) < 16){
        isDone = true;
      }
      speeds = new ChassisSpeeds(yVelocity,xVelocity, rVel);   
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

  public Pose2d getGoalPose(double offsetAngleOffsetAngleRadians, double distanceToTag) {
    double xOffset = distanceToTag * Math.sin(offsetAngleOffsetAngleRadians);
    double yOffset = distanceToTag * Math.cos(offsetAngleOffsetAngleRadians);
    return new Pose2d(xOffset, yOffset, (Rotation2d.fromRadians(offsetAngleOffsetAngleRadians)));
  } 
}
