// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FileVersionException;

import au.grapplerobotics.CanBridge;
import au.grapplerobotics.LaserCan;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  public RobotConfig config;
  private AutoFactory factory;

 private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);

  


  public Robot() {  
    m_robotContainer = new RobotContainer();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.localization.update();
    // System.out.println(m_robotContainer.localization.getPose().getY());
    // System.out.println(m_robotContainer.localization.getPose().getX());
  }

  @Override
  public void robotInit() {
    m_robotContainer.localization.shuffleboard("Localization");
    m_robotContainer.localization.resetFieldPose(new Pose2d(0,0, new Rotation2d()));
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

//   ﻿﻿﻿﻿﻿﻿ pose: Pose2d(Translation2d(X: 7.60, Y: 6.00), Rotation2d(Rads: 0.00, Deg: 0.00)) ﻿
// ﻿﻿﻿﻿﻿﻿ heading: Rotation2d(Rads: 0.00, Deg: 0.00) ﻿
// ﻿﻿﻿﻿﻿﻿ estimator: Pose2d(Translation2d(X: 7.60, Y: 6.00), Rotation2d(Rads: 0.00, Deg: 0.00)) ﻿
// ﻿﻿﻿﻿﻿﻿ vaxis: Rotation2d(Rads: 0.00, Deg: 0.00) ﻿
// ﻿﻿﻿﻿﻿﻿ pose: Pose2d(Translation2d(X: 9.95, Y: 2.05), Rotation2d(Rads: -3.14, Deg: -180.00)) ﻿
// ﻿﻿﻿﻿﻿﻿ heading: Rotation2d(Rads: -3.14, Deg: -180.00) ﻿
// ﻿﻿﻿﻿﻿﻿ estimator: Pose2d(Translation2d(X: 9.95, Y: 2.05), Rotation2d(Rads: -3.14, Deg: -180.00)) ﻿
// ﻿﻿﻿﻿﻿﻿ vaxis: Rotation2d(Rads: -3.14, Deg: -180.00) ﻿

  @Override
  public void autonomousInit() {
    try {
      List<PathPlannerPath> p = PathPlannerAuto.getPathGroupFromAutoFile("LeftSide");
      PathPlannerPath firstPath = p.get(0);
      m_robotContainer.localization.resetFieldPose(firstPath.getStartingHolonomicPose().get());
    } catch (IOException | ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    
    
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

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
