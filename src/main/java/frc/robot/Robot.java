// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatencyType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.utils.ReefScoringPos.ReefPole;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  PowerDistribution pdh;
  public Robot() {  
    m_robotContainer = new RobotContainer();
    pdh = new PowerDistribution(14, ModuleType.kRev);
    m_robotContainer.robotBase.registerPIDCycles(this);
  }

  public Pose2d getPose2d(RobotBase<?> base)
  {

   LimeLight camera_left = base.getVision().getLimelight("limelight-left");
   double dist1 = camera_left.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9];
   double angle1 =  camera_left.getTargetHorizontalOffset() -base.getLocalization().getRelativePose().getRotation().getDegrees() ;
   double x1 = (Math.cos(Math.toRadians(angle1)) * dist1) - Units.inchesToMeters(5);
   double y1 = (Math.sin(Math.toRadians(angle1)) * dist1)- Units.inchesToMeters(10);; 

   
    LimeLight camera_right = base.getVision().getLimelight("limelight-right");
    double dist = camera_right.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9];
    double angle =  camera_right.getTargetHorizontalOffset() -base.getLocalization().getRelativePose().getRotation().getDegrees() ;
    double x2 = (Math.cos(Math.toRadians(angle)) * dist) - Units.inchesToMeters(5);
    double y2 = (Math.sin(Math.toRadians(angle)) * dist) + Units.inchesToMeters(10);
    double x = 0;
    double y = 0;
    if(camera_left.hasValidTarget() && camera_right.hasValidTarget())
    {
      x = (x2 + x1)/2;
      y = (y2 + y1)/2;
    }
    else if(!camera_left.hasValidTarget() && camera_right.hasValidTarget())
    {
      x = (x2);
      y = (y2);
    }
    else if(camera_left.hasValidTarget() && !camera_right.hasValidTarget())
    {
      x = (x1);
      y = (y1);
    }

    Pose2d pose = new Pose2d(-x,y,base.getLocalization().getRelativePose().getRotation());
    if(camera_right.hasValidTarget())
    {
     Pose2d pole = ReefPole.getPoleFromID(camera_right.getAprilTagID(), camera_right).getPose2d();
     base.getLocalization().getField2dObject("flipped").setPose(new Pose2d(pose.getY()+pole.getX(), pose.getX()+pole.getY(), pose.getRotation().plus(pole.getRotation())));
    base.getLocalization().getField2dObject("RobotPose").setPose(pose.relativeTo(ReefPole.getPoleFromID(camera_right.getAprilTagID(), camera_right).getPose2d()));
    base.getLocalization().getField2dObject("Tag").setPose(ReefPole.getPoleFromID(camera_right.getAprilTagID(), camera_right).getPose2d());
    }
    else if(camera_right.hasValidTarget())
    {
     Pose2d pole = ReefPole.getPoleFromID(camera_right.getAprilTagID(), camera_right).getPose2d();
     base.getLocalization().getField2dObject("flipped").setPose(new Pose2d(pose.getY()+pole.getX(), pose.getX()+pole.getY(), pose.getRotation().plus(pole.getRotation())));    
     base.getLocalization().getField2dObject("RobotPose").setPose(pose.relativeTo(ReefPole.getPoleFromID(camera_right.getAprilTagID(), camera_right).getPose2d()));
    base.getLocalization().getField2dObject("Tag").setPose(ReefPole.getPoleFromID(camera_right.getAprilTagID(), camera_right).getPose2d());
    }
    return pose;
  }
   
   


  @Override
  public void robotPeriodic() {

    Pose2d pos = getPose2d(m_robotContainer.robotBase);
    SmartDashboard.putNumber("X Calibration AutoAlign", pos.getX());
    SmartDashboard.putNumber("Y Calibration AutoAlign",  pos.getY());
    SmartDashboard.putNumber("Rot Calibration AutoAlign ", pos.getRotation().getDegrees());

    SmartDashboard.putNumber("X Calibration AutoAlign From Bot", m_robotContainer.robotBase.getLocalization().getRelativePose().getX());
    SmartDashboard.putNumber("Y Calibration AutoAlign From Bot",  m_robotContainer.robotBase.getLocalization().getRelativePose().getY());

    // SmartDashboard.putNumber("W/O TRANSPose X",Units.metersToInches( noTransform.getX()));
    // SmartDashboard.putNumber("W/O TRANSPose Y", Units.metersToInches(noTransform.getY()));
    // SmartDashboard.putNumber("W/O TRANSRotation", noTransform.getRotation().getDegrees());
    // SmartDashboard.putNumber("Tag Rot",    ReefPole.getPoleFromID(m_robotContainer.robotBase.getVision().getLimelight("limelight-left").getAprilTagID(), m_robotContainer.robotBase.getVision().getLimelight("limelight-left")).getPose2d().getRotation().getDegrees());

    CommandScheduler.getInstance().run();
  }

  @Override
  public void robotInit() 
  {
    pdh.clearStickyFaults();
    m_robotContainer.elevator.reset();
    m_robotContainer.robotBase.resetPIDs();
    m_robotContainer.robotBase.getLocalization().resetRelativePose(0, 0, 0);
  }

  @Override
  public void disabledInit() 
  {
    m_robotContainer.elevator.reset();
    m_robotContainer.robotBase.resetPIDs();

    m_robotContainer.superstructure.setSuper(SuperstructureState.Home);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_robotContainer.elevator.reset();
    m_robotContainer.robotBase.resetPIDs();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    Rotation2d offset = Rotation2d.fromDegrees(DriverStation.getAlliance().get().equals(Alliance.Blue) ? 0 : 180);
    m_robotContainer.robotBase.getIMU().setVirtualAxis("driver", m_robotContainer.robotBase.getIMU().getVirtualAxis("field").minus(offset));
  }

  @Override
  public void teleopInit() {
    m_robotContainer.elevator.reset();
    m_robotContainer.robotBase.resetPIDs();
m_robotContainer.robotBase.getLocalization().resetFieldPose(0, 0, 0);
    m_robotContainer.robotBase.getDrivetrain().getRobotSpeeds().stopSpeeds("auto");
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() 
  {
    
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
