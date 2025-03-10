// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.ctre.phoenix6.hardware.Pigeon2;

import au.grapplerobotics.LaserCan;
import ca.frc6390.athena.controllers.EnhancedXboxController;
import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateType;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatencyType;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.ReefScoringPos;
import frc.robot.utils.ReefScoringPos.ReefPole;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReefStrafe extends Command {
  /** Creates a new ReefStrafe. */
  public RobotBase<?> base;
  public LimeLight ll;
  public Pose2d curPose;
  public MedianFilter filter = new MedianFilter(10);
 public PIDController rcontroller = new PIDController(0.025, 0, 0);
 public double thetaMeasurement = 0;
  // public HolonomicDriveController controller 
  // = new HolonomicDriveController
  // (
  public PIDController controller =  new PIDController(0.5, 0, 0);
    // new ProfiledPIDController(0, 0, 0, new Constraints(0, 0))
  // );
  public ReefStrafe(RobotBase<?> base)
  {
   this.base = base;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    rcontroller.enableContinuousInput(-90, 0);
  }

  public Pose2d getBotPoseTagSpace(LimeLight ll)
  {
    double dist = ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9];
    double angle = ll.getTargetHorizontalOffset();
    double x = (Math.cos(Math.toRadians(angle)) * dist);
    double y = (Math.sin(Math.toRadians(angle)) * dist); 
    
    SmartDashboard.putNumber("X1", x);
    SmartDashboard.putNumber("Y1", y);
    SmartDashboard.putNumber("DIST TO TAG", dist);

    SmartDashboard.putNumber("Angle1", angle);   
    
    return new Pose2d(x,y, base.getLocalization().getFieldPose().getRotation());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    ll = base.getVision().getLimelight("limelight-left");

    if(ll.hasValidTarget())
    {
    curPose = getBotPoseTagSpace(ll);
    thetaMeasurement = filter.calculate(ll.getTargetSkew());

    SmartDashboard.putNumber("THETAMeasurement", thetaMeasurement);
    

    }
    else
    {
      thetaMeasurement = 0;
    }
    double Xspeed = 0;//-controller.calculate(curPose.getX(),0);
    double YSpeed = 0;//controller.calculate(curPose.getY(),0);
    double rSpeed = rcontroller.calculate(thetaMeasurement, 0);
    SmartDashboard.putNumber("Xspeed", Xspeed);
    SmartDashboard.putNumber("Yspeed", YSpeed);
    base.getDrivetrain().getRobotSpeeds().setFeedbackSpeeds(0, 0, rSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    base.getDrivetrain().getRobotSpeeds().setFeedbackSpeeds(0,0,0);
    base.getDrivetrain().getRobotSpeeds().stopFeedbackSpeeds();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;
    
  }
}
