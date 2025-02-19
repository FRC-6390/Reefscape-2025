// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.core.RobotLocalization;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.ReefScoringPos.ReefPole;

public class DriveToPoint extends Command {
  public RobotLocalization localization;
  public Pose2d goal;
  public ProfiledPIDController xController = new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));
  public ProfiledPIDController yController = new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));;
  public ProfiledPIDController rotationController = new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));;
  public RobotBase<?> robotBase;
  public boolean isDone;
  public LimeLight limeLight;
  public LimeLight limeLight2;
  public PathPlannerPath sideA;
  public PathPlannerPath sideC;
  public PathPlannerPath sideE;
  public PathPlannerPath sideG;
  public PathPlannerPath sideI;
  public PathPlannerPath sideK;

  

  public DriveToPoint(RobotLocalization localization, RobotBase<?> robotBase, Pose2d goal) {
    this.localization = localization;
    this.robotBase = robotBase;
    isDone = false;
  }

  @Override
  public void initialize() 
  {
    isDone = false;
    limeLight = robotBase.getVision().getCamera("limelight-driver");
    limeLight2 = robotBase.getVision().getCamera("limelight-tag");
  }

  @Override
  public void execute() 
  {

    if(limeLight.hasValidTarget() || limeLight2.hasValidTarget())
    {
      if(limeLight.getAprilTagID() == ReefPole.A.getApriltagId() || limeLight2.getAprilTagID() == ReefPole.A.getApriltagId())
      {
        AutoBuilder.followPath(ReefPole.A.getPath());
      }
      else if(limeLight.getAprilTagID() == ReefPole.C.getApriltagId() || limeLight2.getAprilTagID() == ReefPole.C.getApriltagId())
      {
        AutoBuilder.followPath(ReefPole.C.getPath());
      }
      else if(limeLight.getAprilTagID() == ReefPole.E.getApriltagId()|| limeLight2.getAprilTagID() == ReefPole.E.getApriltagId())
      {
        AutoBuilder.followPath(ReefPole.E.getPath());
      }
      else if(limeLight.getAprilTagID() == ReefPole.G.getApriltagId()|| limeLight2.getAprilTagID() == ReefPole.G.getApriltagId())
      {
        AutoBuilder.followPath(ReefPole.G.getPath());
      }
      else if(limeLight.getAprilTagID() == ReefPole.I.getApriltagId()|| limeLight2.getAprilTagID() == ReefPole.I.getApriltagId())
      {
        AutoBuilder.followPath(ReefPole.I.getPath());
      }
      else if(limeLight.getAprilTagID() == ReefPole.K.getApriltagId()|| limeLight2.getAprilTagID() == ReefPole.K.getApriltagId())
      {
        AutoBuilder.followPath(ReefPole.K.getPath());
      }
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return isDone;
  }
}
