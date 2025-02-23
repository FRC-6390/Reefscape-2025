// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import au.grapplerobotics.LaserCan;
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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.utils.ReefScoringPos.ReefLevel;
import frc.robot.utils.ReefScoringPos.ReefPole;

public class DriveToPoint extends Command {
  public RobotBase<?> robotBase;
  public LimeLight limeLight;
  public boolean isDone;
  public LaserCan las;
  

  public DriveToPoint(RobotBase<?> robotBase, LaserCan las) {
    this.robotBase = robotBase;
    this.las = las;
  }

  @Override
  public void initialize() 
  {
    isDone = false;
    limeLight = robotBase.getCameraFacing(ReefPole.A.getTranslation());
  }

  @Override
  public void execute() 
  {
    if(limeLight.hasValidTarget())
    {
      isDone = true;

    }
  }

  @Override
  public void end(boolean interrupted) 
  {
    if(ReefPole.getPoleFromID(limeLight.getAprilTagID()) != null){
    CommandScheduler.getInstance().schedule(new SequentialCommandGroup(AutoBuilder.followPath(ReefPole.getPoleFromID(limeLight.getAprilTagID()).getPath())));
    
    }

  }

  @Override
  public boolean isFinished() {
    return isDone;
  }
}
