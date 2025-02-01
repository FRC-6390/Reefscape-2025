// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.sound.sampled.Port;

import com.revrobotics.Rev2mDistanceSensor.Unit;

import ca.frc6390.athena.controllers.DebouncedController;
import ca.frc6390.athena.core.RobotVision;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
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
import frc.robot.utils.DistanceSensor;

public class AprilTagAlign extends Command {
  public LimeLight limelight; 
  public SwerveDrivetrain drivetrain;
  public boolean crashed;
  public DebouncedController cont; 
  public PIDController controller = new PIDController(0.025, 0, 0);
  public PIDController xController = new PIDController(0.065, 0, 0.0001);
  public double thetaSpeed = 0;
  public Pose2d targetRed = new Pose2d(13.043, 4.007, new Rotation2d());
  public NetworkTable table;
  public NetworkTableEntry raw;
  public MedianFilter filter = new MedianFilter(10);
  public boolean closeEnough;
  public boolean isDone;
  public ChassisSpeeds speeds;
  public int runTag;
  public DistanceSensor distanceSensor = new DistanceSensor(com.revrobotics.Rev2mDistanceSensor.Port.kOnboard);
  public boolean hasSet;
  public Rotation2d lastYaw;
  public Rotation2d lastRobotYaw;
  public double xMeasurement;
  public double thetaMeasurement;
  public double rVel;
  public ALIGNMODE mode;
  
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

  public AprilTagAlign(LimeLight limeLight, SwerveDrivetrain drivetrain, DebouncedController cont, ALIGNMODE mode ) {
    this.drivetrain = drivetrain; 
    this.cont = cont;
    limelight = limeLight;
    this.mode = mode;
  }

  public AprilTagAlign(LimeLight limeLight, SwerveDrivetrain drivetrain, DebouncedController cont, ALIGNMODE mode, int tagNum) {
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
    crashed = false;
    hasSet = false;
    isDone =false;
    speeds = new ChassisSpeeds();
    rVel = 0;
    lastYaw = new Rotation2d();
    lastRobotYaw  =new Rotation2d();
    thetaMeasurement =0;
    xMeasurement = 0;
    speeds =new ChassisSpeeds();
    runTag = -1;
    distanceSensor.setEnabled(true);
    distanceSensor.setAutomaticMode(true);
    
  }


  @Override
  public void execute() 
  {
    SmartDashboard.putNumber("Distance",distanceSensor.getRange(Unit.kInches));
   if(DriverStation.isTeleop())
  {
    if(limelight.hasValidTarget()){
      if(!hasSet)
      {
        System.out.println("Setting Tag");
        hasSet = true;
        runTag = ((int)limelight.getAprilTagID());
      }
      

      if(((int)limelight.getAprilTagID()) == runTag) 
      {
      thetaMeasurement = -filter.calculate(limelight.getPoseEstimate(PoseEstimateType.TARGET_POSE_ROBOT_SPACE).getRaw()[4]);
      lastYaw = Rotation2d.fromDegrees(thetaMeasurement);
      lastRobotYaw = Rotation2d.fromRadians(MathUtil.angleModulus(drivetrain.getIMU().getYaw().getRadians()));
      xMeasurement = limelight.getTargetHorizontalOffset();
       rVel = -controller.calculate(thetaMeasurement, 0);
      }
      else
      {
        double rot = lastRobotYaw.getDegrees() + lastYaw.getDegrees();
        // thetaMeasurement -= rot;
        rVel = controller.calculate(MathUtil.angleModulus(drivetrain.getIMU().getYaw().getRadians()) * 180/Math.PI, rot);
      }
    }
    else
    {
      double rot = lastRobotYaw.getDegrees() + lastYaw.getDegrees();
      // thetaMeasurement -= rot;
      rVel = controller.calculate(MathUtil.angleModulus(drivetrain.getIMU().getYaw().getRadians()) * 180/Math.PI, rot);
    }

    if(hasSet) {
       
      double xVelocity = mode.get() * xController.calculate(xMeasurement);
      // double rotationalVelocity = -controller.calculate(thetaMeasurement);
      speeds = new ChassisSpeeds(mode.get(),xVelocity, rVel);   
      drivetrain.feedbackSpeeds(speeds);
    }
  }
  else
  {
    System.out.print("EXECUTING");
    if(limelight.hasValidTarget()){
      if(!hasSet)
      {
        System.out.println("Setting Tag");
        hasSet = true;
        runTag = ((int)limelight.getAprilTagID());
      }

      if(((int)limelight.getAprilTagID()) == runTag) 
      {
      thetaMeasurement = -filter.calculate(limelight.getPoseEstimate(PoseEstimateType.TARGET_POSE_ROBOT_SPACE).getRaw()[4]);
      lastYaw = Rotation2d.fromDegrees(thetaMeasurement);
      lastRobotYaw = Rotation2d.fromRadians(MathUtil.angleModulus(drivetrain.getIMU().getYaw().getRadians()));
      xMeasurement = limelight.getTargetHorizontalOffset();
      rVel =  -controller.calculate(thetaMeasurement, 0);
      System.out.println(limelight.getTargetArea());
      if(limelight.getTargetArea() > 10)
      {
        closeEnough = true;
      }
      }
    }
    if(!limelight.hasValidTarget() && closeEnough && distanceSensor.getRange(Unit.kInches) < 6 && distanceSensor.isRangeValid())
    {
      isDone = true;
    }
    if(hasSet) {
       
      double xVelocity =  mode.get() * xController.calculate(xMeasurement);
      // double rotationalVelocity = -controller.calculate(thetaMeasurement);
      speeds = new ChassisSpeeds(mode.get(),xVelocity, rVel);   
      drivetrain.feedbackSpeeds(speeds);
    }
    
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    drivetrain.drive(new ChassisSpeeds(0,0,0));
    drivetrain.feedbackSpeeds(new ChassisSpeeds(0,0,0));
    SmartDashboard.putNumber("Is Running", 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(DriverStation.isTeleop())
    {
    return false;
    }
    else
    {
    return isDone;
    }
  }

  public Pose2d getGoalPose(double offsetAngleOffsetAngleRadians, double distanceToTag, Rotation2d botAngle) {
    double xOffset = distanceToTag * Math.sin(offsetAngleOffsetAngleRadians);
    double yOffset = distanceToTag * Math.cos(offsetAngleOffsetAngleRadians);
    return new Pose2d(xOffset, yOffset, botAngle.plus(Rotation2d.fromRadians(offsetAngleOffsetAngleRadians)));
  } 
}
