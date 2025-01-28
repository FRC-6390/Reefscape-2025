// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.signals.RobotEnableValue;

import ca.frc6390.athena.controllers.DebouncedController;
import ca.frc6390.athena.core.RobotLocalization;
import ca.frc6390.athena.core.RobotVision;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimelightConfig;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class AprilTagAlign extends Command {
  public LimeLight limelight; 
  public SwerveDrivetrain drivetrain;
  public DebouncedController cont; 
  // public PIDController controller = new PIDController(0.025, 0, 0);
  // public PIDController xController = new PIDController(1.225, 0, 0);
  public PIDController controller = new PIDController(0.025, 0, 0);
  public PIDController xController = new PIDController(0.07, 0, 0.0001);
  public double thetaSpeed = 0;
  public Pose2d targetRed = new Pose2d(13.043, 4.007, new Rotation2d());
  public RobotVision vision;
  public NetworkTable table;
  public NetworkTableEntry raw;
  public MedianFilter filter = new MedianFilter(10);
  public boolean closeEnough;
  public boolean isDone;
  public ChassisSpeeds speeds;
  public int runTag;
  public boolean hasSet;
  public Rotation2d lastYaw;
  public Rotation2d lastRobotYaw;
  public double xMeasurement;
  public double thetaMeasurement;
  public double rVel;

  public AprilTagAlign(RobotVision vision, SwerveDrivetrain drivetrain, DebouncedController cont) {
    this.drivetrain = drivetrain; 
    this.vision = vision;
    this.cont = cont;
    limelight = vision.getCamera("limelight-driver");
  }

  @Override
  public void initialize() 
  {
    closeEnough = false;
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
    // controller.enableContinuousInput(-Math.PI, Math.PI);
    controller.setTolerance(0.3);
    table = NetworkTableInstance.getDefault().getTable("limelight-driver");
    raw = table.getEntry("rawfiducials");    
  }


  @Override
  public void execute() 
  {
    SmartDashboard.putNumber("Tag",runTag);
    if(limelight.hasValidTarget()){
      SmartDashboard.putBoolean("Sim", ((int)limelight.getAprilTagID()) == runTag);
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
      SmartDashboard.putNumber("Yaw",thetaMeasurement);
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
      SmartDashboard.putNumber("Des Rot", rot);
      SmartDashboard.putNumber("Hedingng", MathUtil.angleModulus(drivetrain.getIMU().getYaw().getRadians()) * 180/Math.PI);
      // thetaMeasurement -= rot;
      rVel = controller.calculate(MathUtil.angleModulus(drivetrain.getIMU().getYaw().getRadians()) * 180/Math.PI, rot);
    }

    if(hasSet) {
      SmartDashboard.putNumber("Last Yaw", lastYaw.getDegrees());
      SmartDashboard.putNumber("Last Robot Yaw", lastRobotYaw.getDegrees());
      SmartDashboard.putNumber("Cur Heading", drivetrain.getIMU().getYaw().getDegrees());
       
      double xVelocity = -xController.calculate(xMeasurement);
      // double rotationalVelocity = -controller.calculate(thetaMeasurement);
      speeds = new ChassisSpeeds(-1,xVelocity, rVel);   
      drivetrain.drive(speeds);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    drivetrain.drive(new ChassisSpeeds(0,0,0));
    drivetrain.feedbackSpeeds(new ChassisSpeeds(0,0,0));
    drivetrain.drive(drivetrain.getDriveSpeeds());
    SmartDashboard.putNumber("Is Running", 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
