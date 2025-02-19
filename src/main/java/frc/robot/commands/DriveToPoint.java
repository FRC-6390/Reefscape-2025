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
import frc.robot.utils.ReefScoringPos.ReefLevel;
import frc.robot.utils.ReefScoringPos.ReefPole;

public class DriveToPoint extends Command {
  public RobotLocalization localization;
  public Pose2d goal;
  public RobotBase<?> robotBase;
  public LimeLight limeLight;
  

  public DriveToPoint(RobotLocalization localization, RobotBase<?> robotBase, Pose2d goal) {
    this.localization = localization;
    this.robotBase = robotBase;
  }

  @Override
  public void initialize() 
  {
    limeLight = robotBase.getCameraFacing(ReefPole.A.getTranslation());
  }

  @Override
  public void execute() 
  {
    if(limeLight.hasValidTarget())
    {
      AutoBuilder.followPath(ReefPole.getPoleFromID((int)limeLight.getAprilTagID()).getPath());
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
