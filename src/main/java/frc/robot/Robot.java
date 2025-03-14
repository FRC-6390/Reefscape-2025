// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.PassiveAlign;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.utils.ReefScoringPos;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;

  public Robot() {  
    
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
      Pose2d relativePose = m_robotContainer.robotBase.getLocalization().getRelativePose();
    SmartDashboard.putNumber("Skew", m_robotContainer.robotBase.getVision().getLimelight("limelight-right").getTargetSkew());
    if(m_robotContainer.lasLeft.getMeasurement() != null) SmartDashboard.putNumber("Las Left",m_robotContainer.lasLeft.getMeasurement().distance_mm);
    if(m_robotContainer.lasRight.getMeasurement() != null) SmartDashboard.putNumber("Las Right",m_robotContainer.lasRight.getMeasurement().distance_mm);

  }

  @Override
  public void robotInit() 
  {
    
    // CanBridge.runTCP();
  }

  @Override
  public void disabledInit() 
  {

  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
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
    m_robotContainer.robotBase.getDrivetrain().getRobotSpeeds().stopAutoSpeeds();
    // m_robotContainer.elevator.reset();
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
